package org.team1540.swervedrive.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.Robot;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.CharacterizationCommands;
import org.team1540.swervedrive.generated.TunerConstants;
import org.team1540.swervedrive.util.*;

public class Drivetrain extends SubsystemBase {
    static final double ODOMETRY_FREQUENCY = 250.0;

    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    public static final double MAX_TOTAL_MODULE_FORCES = DCMotor.getKrakenX60Foc(4)
                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio)
                    .getTorque(TunerConstants.FrontLeft.SlipCurrent)
            / TunerConstants.FrontLeft.WheelRadius;

    public static final double MAX_LINEAR_SPEED_MPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_LINEAR_ACCELERATION_MPS2 = MAX_TOTAL_MODULE_FORCES / Constants.ROBOT_MASS_KG;

    public static final double MAX_ANGULAR_SPEED_RADS_PER_SEC = MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;
    public static final double MAX_ANGULAR_ACCELERATION_RADS_PER_SEC2 =
            MAX_TOTAL_MODULE_FORCES * DRIVE_BASE_RADIUS / Constants.ROBOT_MOI_KG_M2;
    public static final double MAX_STEER_SPEED_RADS_PER_SEC =
            DCMotor.getFalcon500(1).withReduction(TunerConstants.FrontLeft.SteerMotorGearRatio).freeSpeedRadPerSec;

    public static final double WHEEL_COF = 1.2;

    private static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY),
    };

    private static boolean hasInstance;
    static final Lock odometryLock = new ReentrantLock();

    private static final LoggedTunableNumber translationKP = new LoggedTunableNumber("Drivetrain/Translation/kP", 4.0);
    private static final LoggedTunableNumber translationKI = new LoggedTunableNumber("Drivetrain/Translation/kI", 0.0);
    private static final LoggedTunableNumber translationKD = new LoggedTunableNumber("Drivetrain/Translation/kD", 0.0);

    private static final LoggedTunableNumber headingKP = new LoggedTunableNumber("Drivetrain/Heading/kP", 6.5);
    private static final LoggedTunableNumber headingKI = new LoggedTunableNumber("Drivetrain/Heading/kI", 0.0);
    private static final LoggedTunableNumber headingKD = new LoggedTunableNumber("Drivetrain/Heading/kD", 0.0);

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private Rotation2d fieldOrientationOffset = Rotation2d.kZero;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4]; // For odometry delta filtering
    private double lastOdometryUpdateTime = 0.0;

    private boolean isFFCharacterizing = false;
    private double ffCharacterizationInput = 0.0;

    @AutoLogOutput(key = "Drivetrain/DesiredSpeeds")
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private final TrajectoryController trajectoryController = new TrajectoryController(
            translationKP.get(),
            translationKI.get(),
            translationKD.get(),
            headingKP.get(),
            headingKI.get(),
            headingKD.get());

    private final ProfiledPIDController headingController = new ProfiledPIDController(
            headingKP.get(),
            headingKI.get(),
            headingKD.get(),
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADS_PER_SEC, MAX_ANGULAR_ACCELERATION_RADS_PER_SEC2));

    private final Alert gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.kWarning);

    public Drivetrain(
            GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        if (hasInstance) throw new IllegalStateException("Instance of drivetrain already exists");
        hasInstance = true;

        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, Module.MountPosition.FL, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, Module.MountPosition.FR, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, Module.MountPosition.BL, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, Module.MountPosition.BR, TunerConstants.BackRight);

        // Start odometry thread
        OdometryThread.getInstance().start();

        RobotState.getInstance().configurePoseEstimation(kinematics);

        for (int i = 0; i < 4; i++) {
            lastModulePositions[i] = modules[i].getPosition();
        }

        headingController.setTolerance(Math.toRadians(1.0));
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (Module module : modules) module.periodic();
        odometryLock.unlock();

        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        int rejectedSamples = 0;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
            }

            // Filter odometry data based on wheel deltas
            boolean acceptMeasurement = true;
            double dt = sampleTimestamps[i] - lastOdometryUpdateTime;
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                double velocity = moduleDeltas[moduleIndex].distanceMeters / dt;
                double turnVelocity = modulePositions[moduleIndex]
                                .angle
                                .minus(lastModulePositions[moduleIndex].angle)
                                .getRadians()
                        / dt;
                if (Math.abs(velocity) > MAX_LINEAR_SPEED_MPS * 2
                        || Math.abs(turnVelocity) > MAX_STEER_SPEED_RADS_PER_SEC * 2) {
                    acceptMeasurement = false;
                    break;
                }
            }
            // Accept measurements if delta is not too large
            if (acceptMeasurement) {
                if (gyroInputs.connected) rawGyroRotation = gyroInputs.odometryYawPositions[i];
                else {
                    Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
                    rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
                }
                RobotState.getInstance().addOdometryObservation(modulePositions, rawGyroRotation, sampleTimestamps[i]);
                lastModulePositions = modulePositions;
                lastOdometryUpdateTime = sampleTimestamps[i];
            } else {
                rejectedSamples++;
            }
        }
        Logger.recordOutput("Odometry/RejectedSamples", rejectedSamples);

        // Update robot velocities
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        speeds.omegaRadiansPerSecond =
                gyroInputs.connected ? gyroInputs.yawVelocityRadPerSec : speeds.omegaRadiansPerSecond;
        RobotState.getInstance().addVelocityData(speeds);

        if (DriverStation.isEnabled()) {
            // Run modules based on current drive mode
            if (isFFCharacterizing) {
                for (Module module : modules) module.runCharacterization(ffCharacterizationInput);
            } else {
                SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(desiredSpeeds);
                for (int i = 0; i < 4; i++) {
                    modules[i].runSetpoint(setpointStates[i]);
                }
                Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", setpointStates);
            }
        } else {
            for (Module module : modules) module.stop(); // Stop modules when disabled
            Logger.recordOutput(
                    "Drivetrain/SwerveStates/Setpoints",
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState());
        }

        // Update gyro alerts
        gyroDisconnected.set(Robot.isReal() && !gyroInputs.connected);

        // Update tunable numbers
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> trajectoryController.setTranslationPID(
                        translationKP.get(), translationKI.get(), translationKD.get()),
                translationKP,
                translationKI,
                translationKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    headingController.setPID(headingKP.get(), headingKI.get(), headingKD.get());
                    trajectoryController.setHeadingPID(headingKP.get(), headingKI.get(), headingKD.get());
                },
                headingKP,
                headingKI,
                headingKD);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    private void runVelocity(ChassisSpeeds speeds) {
        desiredSpeeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_SECS);
    }

    public void followTrajectory(SwerveSample trajectorySample) {
        RobotState.getInstance().setTrajectoryTarget(trajectorySample.getPose());
        runVelocity(trajectoryController.calculate(RobotState.getInstance().getRobotPose(), trajectorySample));
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) headings[i] = MODULE_TRANSLATIONS[i].getAngle();
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Zeroes field-oriented drive to the direction the robot is facing */
    public void zeroFieldOrientationManual() {
        fieldOrientationOffset = rawGyroRotation;
    }

    /** Zeroes field-oriented drive to the field based on the calculated odometry position */
    public void zeroFieldOrientation() {
        fieldOrientationOffset = rawGyroRotation.minus(
                AllianceFlipUtil.shouldFlip()
                        ? RobotState.getInstance().getRotation().plus(Rotation2d.k180deg)
                        : RobotState.getInstance().getRotation());
    }

    /** Sets the brake mode of all modules */
    public void setBrakeMode(boolean enabled) {
        for (Module module : modules) module.setBrakeMode(enabled);
    }

    /** Orients all modules forward and applies the specified voltage to the drive motors */
    private void runFFCharacterization(double volts) {
        ffCharacterizationInput = volts;
        isFFCharacterizing = true;
    }

    /** Ends characterization and returns to default drive behavior */
    private void endFFCharacterization() {
        ffCharacterizationInput = 0;
        isFFCharacterizing = false;
    }

    /** Returns the average velocity of each module in rot/s */
    private double getFFCharacterizationVelocity() {
        double driveVelocityAverage = 0;
        for (Module module : modules) driveVelocityAverage += module.getFFCharacterizationVelocity();
        return driveVelocityAverage / modules.length;
    }

    /** Returns the position of each module in radians */
    private double[] getWheelRadiusCharacterizationPositions() {
        return Arrays.stream(modules)
                .mapToDouble(Module::getWheelRadiusCharacterizationPosition)
                .toArray();
    }

    /** Returns the module states (turn angles and drive velocities) for all the modules. */
    @AutoLogOutput(key = "Drivetrain/SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public Command percentDriveCommand(
            Supplier<Translation2d> linearPercent, DoubleSupplier omegaPercent, BooleanSupplier fieldRelative) {
        return Commands.run(
                        () -> {
                            var speeds = new ChassisSpeeds(
                                    linearPercent.get().getX() * MAX_LINEAR_SPEED_MPS,
                                    linearPercent.get().getY() * MAX_LINEAR_SPEED_MPS,
                                    omegaPercent.getAsDouble() * MAX_ANGULAR_SPEED_RADS_PER_SEC);
                            if (fieldRelative.getAsBoolean()) {
                                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                        speeds, rawGyroRotation.minus(fieldOrientationOffset));
                            }
                            runVelocity(speeds);
                        },
                        this)
                .finallyDo(this::stop);
    }

    public Command teleopDriveCommand(XboxController controller, BooleanSupplier fieldRelative) {
        return percentDriveCommand(
                () -> JoystickUtil.getSquaredJoystickTranslation(-controller.getLeftY(), -controller.getLeftX(), 0.1),
                () -> JoystickUtil.squaredSmartDeadzone(-controller.getRightX(), 0.1),
                fieldRelative);
    }

    public Command teleopDriveWithHeadingCommand(
            XboxController controller,
            Supplier<Rotation2d> heading,
            DoubleSupplier angularVelocityFFRadsPerSec,
            BooleanSupplier fieldRelative) {
        return percentDriveCommand(
                        () -> JoystickUtil.getSquaredJoystickTranslation(
                                -controller.getLeftY(), -controller.getLeftX(), 0.1),
                        () -> (headingController.calculate(
                                                RobotState.getInstance()
                                                        .getRotation()
                                                        .getRadians(),
                                                new TrapezoidProfile.State(
                                                        heading.get().getRadians(),
                                                        angularVelocityFFRadsPerSec.getAsDouble()))
                                        + angularVelocityFFRadsPerSec.getAsDouble())
                                / MAX_ANGULAR_SPEED_RADS_PER_SEC,
                        fieldRelative)
                .beforeStarting(() -> headingController.reset(
                        RobotState.getInstance().getRotation().getRadians(),
                        RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond))
                .alongWith(Commands.run(() -> Logger.recordOutput("Drivetrain/HeadingGoal", heading.get())))
                .until(() -> Math.abs(controller.getRightX()) >= 0.1);
    }

    public Command teleopDriveWithHeadingCommand(
            XboxController controller, Supplier<Rotation2d> heading, BooleanSupplier fieldRelative) {
        return teleopDriveWithHeadingCommand(controller, heading, () -> 0.0, fieldRelative);
    }

    public Command feedforwardCharacterization() {
        return CharacterizationCommands.feedforward(
                        this::runFFCharacterization, this::getFFCharacterizationVelocity, this)
                .finallyDo(this::endFFCharacterization);
    }

    public Command wheelRadiusCharacterization() {
        return CharacterizationCommands.wheelRadius(
                input -> runVelocity(new ChassisSpeeds(0.0, 0.0, input)),
                () -> rawGyroRotation.getRadians(),
                this::getWheelRadiusCharacterizationPositions,
                this);
    }

    public static Drivetrain createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real drivetrain on simulated robot", false);
        return new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
    }

    public static Drivetrain createSim() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated drivetrain on real robot", false);

        var simConfig = DriveTrainSimulationConfig.Default()
                .withRobotMass(Kilograms.of(Constants.ROBOT_MASS_KG))
                .withCustomModuleTranslations(MODULE_TRANSLATIONS)
                .withBumperSize(
                        Meters.of(Constants.BUMPER_LENGTH_X_METERS), Meters.of(Constants.BUMPER_LENGTH_Y_METERS))
                .withGyro(() -> new GyroSimulation(0.12 / 120, 0.02))
                .withSwerveModule(() -> new SwerveModuleSimulation(new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getFalcon500(1),
                        TunerConstants.FrontLeft.DriveMotorGearRatio,
                        TunerConstants.FrontLeft.SteerMotorGearRatio,
                        Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                        Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                        Meters.of(TunerConstants.FrontLeft.WheelRadius),
                        KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                        WHEEL_COF)));
        var driveSim = new SwerveDriveSimulation(simConfig, Pose2d.kZero);

        RobotState.getInstance().configureDriveSim(driveSim);
        return new Drivetrain(
                new GyroIOSim(driveSim.getGyroSimulation()),
                new ModuleIOSim(TunerConstants.FrontLeft, driveSim.getModules()[0]),
                new ModuleIOSim(TunerConstants.FrontRight, driveSim.getModules()[1]),
                new ModuleIOSim(TunerConstants.BackLeft, driveSim.getModules()[2]),
                new ModuleIOSim(TunerConstants.BackRight, driveSim.getModules()[3]));
    }

    public static Drivetrain createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy drivetrain on real robot", false);
        return new Drivetrain(
                new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    }
}
