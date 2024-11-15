package org.team1540.swervedrive.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.Robot;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.FeedForwardCharacterization;
import org.team1540.swervedrive.commands.WheelRadiusCharacterization;
import org.team1540.swervedrive.util.*;
import org.team1540.swervedrive.util.swerve.ModuleConfig;
import org.team1540.swervedrive.util.swerve.ModuleLimits;
import org.team1540.swervedrive.util.swerve.SwerveSetpointGenerator;
import org.team1540.swervedrive.util.swerve.SwerveSetpointGenerator.SwerveSetpoint;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static org.team1540.swervedrive.util.math.EqualsUtil.*;
import static org.team1540.swervedrive.util.math.GeomUtil.*;

public class Drivetrain extends SubsystemBase {
    public static final String CAN_BUS = "swerve";
    public static final double ODOMETRY_FREQUENCY = 250.0;

    // Drive and turn closed loop gains
    public static final ClosedLoopConfig DRIVE_VELOCITY_GAINS =
            Robot.isReal()
                    ? new ClosedLoopConfig(0.3, 0, 0, 0.258, 0.804)
                    : new ClosedLoopConfig(0.3, 0.0, 0.0, 0.00666, 0.83397);
    public static final ClosedLoopConfig TURN_POSITION_GAINS =
            Robot.isReal()
                    ? new ClosedLoopConfig(100, 0, 0.5)
                    : new ClosedLoopConfig(100.0, 0.0, 0.0);

    // Module configuration
    public static final ModuleConfig MODULE_CONFIG =
            new ModuleConfig(
                    (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0),
                    150.0 / 7.0,
                    true,
                    100,
                    DRIVE_VELOCITY_GAINS,
                    TURN_POSITION_GAINS,
                    CAN_BUS);
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    // Module IDs
    public static final int FL_ID = 1;
    public static final int FR_ID = 2;
    public static final int BL_ID = 3;
    public static final int BR_ID = 4;

    // Kinematic constants
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
    public static final double MAX_LINEAR_ACCELERATION = Units.feetToMeters(75.0);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(19.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(19.75);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double MAX_ANGULAR_ACCEL = MAX_LINEAR_ACCELERATION / DRIVE_BASE_RADIUS;

    public static final ModuleLimits MODULE_LIMITS =
            new ModuleLimits(MAX_LINEAR_SPEED, MAX_LINEAR_ACCELERATION, Units.degreesToRadians(1700));
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(getModuleTranslations());

    public static final LoggedTunableNumber headingKP =
            new LoggedTunableNumber("Drivetrain/HeadingController/KP", 0.5);
    public static final LoggedTunableNumber headingKD =
            new LoggedTunableNumber("Drivetrain/HeadingController/KD", 0.0);
    public static final LoggedTunableNumber maxHeadingVelocityMultiplier =
            new LoggedTunableNumber("Drivetrain/HeadingController/MaxVelocityMultiplier", 0.8);
    public static final LoggedTunableNumber maxHeadingAccelerationMultiplier =
            new LoggedTunableNumber("Drivetrain/HeadingController/MaxAccelerationMultiplier", 0.8);

    public enum DriveMode {
        /** Standard drive mode, driving according to desired chassis speeds */
        DEFAULT,

        /** Characterizing drive motor velocity feedforwards (driving motors at specific voltages) */
        FF_CHARACTERIZATION,

        /** Spinning in a circle and using gyro rotation to characterize wheel radius */
        WHEEL_RADIUS_CHARACTERIZATION
    }

    private static boolean hasInstance;

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private Rotation2d fieldOrientationOffset = new Rotation2d();

    // Store previous positions and time for filtering odometry data
    private SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private double lastOdometryUpdateTime = 0.0;

    @AutoLogOutput(key = "Drivetrain/CurrentDriveMode")
    private DriveMode currentDriveMode = DriveMode.DEFAULT;
    private double characterizationInput;
    private boolean forceModuleRotation = false;

    @AutoLogOutput(key = "Drivetrain/DesiredSpeeds")
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private SwerveSetpoint currentSetpoint =
            new SwerveSetpoint(new ChassisSpeeds(),
                    new SwerveModuleState[] {
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState(),
                            new SwerveModuleState()
                    });
    private Supplier<Rotation2d> headingGoal = null;

    private final SwerveSetpointGenerator setpointGenerator =
            new SwerveSetpointGenerator(KINEMATICS, getModuleTranslations());
    private final ProfiledPIDController headingController =
            new ProfiledPIDController(
                    headingKP.get(),
                    0.0,
                    headingKD.get(),
                    new TrapezoidProfile.Constraints(
                            MAX_ANGULAR_SPEED * maxHeadingVelocityMultiplier.get(),
                            MAX_ANGULAR_ACCEL * maxHeadingAccelerationMultiplier.get()));

    private final Alert gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.WARNING);

    public Drivetrain(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        if (hasInstance) throw new IllegalStateException("Instance of drivetrain already exists");
        hasInstance = true;

        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                RobotState.getInstance()::getRobotPose,
                RobotState.getInstance()::resetPose,
                () -> RobotState.getInstance().getRobotVelocity(),
                this::runVelocity,
                new HolonomicPathFollowerConfig(
                        MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
                AllianceFlipUtil::shouldFlip,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> RobotState.getInstance().setActiveTrajectory(activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(RobotState.getInstance()::setCurrentTrajectoryTarget);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (Module module : modules) module.updateInputs();
        odometryLock.unlock();
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);
        for (Module module : modules) module.periodic();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) for (Module module : modules) module.stop();

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
            }

            // Filter odometry data based on wheel deltas
            boolean acceptMeasurement = true;
            if (lastModulePositions != null) {
                double dt = sampleTimestamps[i] - lastOdometryUpdateTime;
                for (int j = 0; j < modules.length; j++) {
                    double velocity = moduleDeltas[j].distanceMeters / dt;
                    double turnVelocity = modulePositions[j].angle.minus(lastModulePositions[i].angle).getRadians() / dt;
                    if (Math.abs(velocity) > MODULE_LIMITS.maxDriveVelocity() * 5
                            || Math.abs(turnVelocity) > MODULE_LIMITS.maxTurnVelocity() * 5) {
                        acceptMeasurement = false;
                        break;
                    }
                }
            }
            // Accept measurements if delta is not too large
            if (acceptMeasurement) {
                Rotation2d gyroRotation = gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null;
                RobotState.getInstance().addOdometryObservation(modulePositions, gyroRotation, sampleTimestamps[i]);
                lastModulePositions = modulePositions;
                lastOdometryUpdateTime = sampleTimestamps[i];
            }
        }

        // Update robot velocities
        ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(getModuleStates());
        speeds.omegaRadiansPerSecond = gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : speeds.omegaRadiansPerSecond;
        RobotState.getInstance().addVelocityData(speeds);

        SwerveModuleState[] setpointStates = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        // Run modules based on current drive mode
        switch (currentDriveMode) {
            case FF_CHARACTERIZATION -> {
                for (Module module : modules) module.runCharacterization(characterizationInput);
            }
            case WHEEL_RADIUS_CHARACTERIZATION ->
                    setpointStates =
                            KINEMATICS.toSwerveModuleStates(
                                    new ChassisSpeeds(0, 0, characterizationInput));
            default -> {
                if (headingGoal != null) {
                    Logger.recordOutput("Drivetrain/HeadingGoal", headingGoal.get());
                    desiredSpeeds.omegaRadiansPerSecond =
                            headingController.calculate(
                                    RobotState.getInstance().getRotation().getRadians(),
                                    headingGoal.get().getRadians());
                }
                Logger.recordOutput(
                        "Drivetrain/SwerveStates/DesiredSetpoints", KINEMATICS.toSwerveModuleStates(desiredSpeeds));
                // Generate kinematically feasible setpoint
                currentSetpoint =
                        setpointGenerator.generateSetpoint(
                                MODULE_LIMITS,
                                currentSetpoint,
                                desiredSpeeds,
                                forceModuleRotation,
                                Constants.LOOP_PERIOD_SECS);
                setpointStates = currentSetpoint.moduleStates();
            }
        }
        if (currentDriveMode != DriveMode.FF_CHARACTERIZATION) {
            for (int i = 0; i < 4; i++) {
                setpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
            }
            Logger.recordOutput("Drivetrain/SwerveStates/OptimizedSetpoints", setpointStates);
        }

        // Update tunable numbers
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> headingController.setPID(headingKP.get(), 0.0, headingKD.get()), headingKP, headingKD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> headingController.setConstraints(
                        new TrapezoidProfile.Constraints(
                                MAX_ANGULAR_SPEED * maxHeadingVelocityMultiplier.get(),
                                MAX_ANGULAR_ACCEL * maxHeadingAccelerationMultiplier.get())),
                maxHeadingVelocityMultiplier,
                maxHeadingAccelerationMultiplier);


        // Update gyro alerts
        gyroDisconnected.set(Robot.isReal() && !gyroInputs.connected);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        if (!epsilonEquals(toTwist2d(speeds), new Twist2d())) forceModuleRotation = false;
        desiredSpeeds = ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_SECS);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) headings[i] = getModuleTranslations()[i].getAngle();
        KINEMATICS.resetHeadings(headings);
        forceModuleRotation = true;
        stop();
    }

    /** Zeroes field-oriented drive to the direction the robot is facing */
    public void zeroFieldOrientationManual() {
        fieldOrientationOffset = RobotState.getInstance().getRawGyroRotation();
    }

    /** Zeroes field-oriented drive to the field based on the calculated odometry position */
    public void zeroFieldOrientation() {
        fieldOrientationOffset =
                RobotState.getInstance().getRawGyroRotation().minus(AllianceFlipUtil.shouldFlip()
                        ? RobotState.getInstance().getRotation().plus(Rotation2d.fromDegrees(180))
                        : RobotState.getInstance().getRotation());
    }

    public void setHeadingGoal(Supplier<Rotation2d> headingGoal) {
        this.headingGoal = headingGoal;
    }

    public void clearHeadingGoal() {
        headingGoal = null;
    }

    public boolean atHeadingGoal() {
        return headingGoal == null || headingController.atGoal();
    }

    /** Orients all modules forward and applies the specified voltage to the drive motors */
    public void runFFCharacterization(double volts) {
        characterizationInput = volts;
        currentDriveMode = DriveMode.FF_CHARACTERIZATION;
    }

    /** Spins the robot in a circle at the specified angular velocity to characterize wheel radius */
    public void runWheelRadiusCharacterization(double omega) {
        characterizationInput = omega;
        currentDriveMode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
    }

    /** Ends characterization and returns to default drive behavior */
    public void endCharacterization() {
        characterizationInput = 0;
        currentDriveMode = DriveMode.DEFAULT;
    }

    /** Returns the average velocity of each module in rot/s */
    public double getFFCharacterizationVelocity() {
        double driveVelocityAverage = 0;
        for (Module module : modules) driveVelocityAverage += module.getFFCharacterizationVelocity();
        return driveVelocityAverage / modules.length;
    }

    /** Returns the position of each module in radians */
    public double[] getWheelRadiusCharacterizationPositions() {
        return Arrays.stream(modules).mapToDouble(Module::getWheelRadiusCharacterizationPosition).toArray();
    }

    /** Returns the module states (turn angles and drive velocities) for all the modules. */
    @AutoLogOutput(key = "Drivetrain/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
        };
    }

    /** Returns a command that drives the robot based on joystick inputs */
    public Command teleopDriveCommand(
            DoubleSupplier controllerX,
            DoubleSupplier controllerY,
            DoubleSupplier controllerOmega,
            BooleanSupplier fieldRelative) {
        return Commands.run(() -> {
            double xPercent = controllerX.getAsDouble();
            double yPercent = controllerY.getAsDouble();
            double omega = JoystickUtil.smartDeadzone(controllerOmega.getAsDouble(), 0.1);

            double linearMagnitude = JoystickUtil.smartDeadzone(Math.hypot(xPercent, yPercent), 0.1);
            Rotation2d linearDirection = new Rotation2d(xPercent, yPercent);
            Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

            runVelocity(
                    fieldRelative.getAsBoolean()
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            linearVelocity.getX() * MAX_LINEAR_SPEED,
                            linearVelocity.getY() * MAX_LINEAR_SPEED,
                            omega * MAX_ANGULAR_SPEED,
                            RobotState.getInstance().getRawGyroRotation().minus(fieldOrientationOffset))
                            : new ChassisSpeeds(
                            linearVelocity.getX() * MAX_LINEAR_SPEED,
                            linearVelocity.getY() * MAX_LINEAR_SPEED,
                            omega * MAX_ANGULAR_SPEED));
        }, this).finallyDo(this::stop);
    }

    /** Returns a command that overrides all drive control and rotates it to face the specified heading */
    public Command snapToHeading(Supplier<Rotation2d> heading) {
        return Commands.runOnce(() -> setHeadingGoal(heading), this)
                .andThen(Commands.waitUntil(this::atHeadingGoal))
                .finallyDo(this::clearHeadingGoal);
    }

    /**
     * Returns a command that overrides rotational control to keep the drive locked to the specified heading while
     * running
     */
    public Command lockHeading(Supplier<Rotation2d> heading) {
        return Commands.startEnd(
                () -> setHeadingGoal(heading),
                this::clearHeadingGoal);
    }

    public Command feedforwardCharacterization() {
        return new FeedForwardCharacterization(
                this,
                this::runFFCharacterization,
                this::getFFCharacterizationVelocity
        ).finallyDo(this::endCharacterization);
    }

    public Command wheelRadiusCharacterization(WheelRadiusCharacterization.Direction direction) {
        return new WheelRadiusCharacterization(this, direction).finallyDo(this::endCharacterization);
    }

    public static Drivetrain createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real drivetrain on simulated robot", false);
        return new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(MODULE_CONFIG, FL_ID),
                new ModuleIOTalonFX(MODULE_CONFIG, FR_ID),
                new ModuleIOTalonFX(MODULE_CONFIG, BL_ID),
                new ModuleIOTalonFX(MODULE_CONFIG, BR_ID)
        );
    }

    public static Drivetrain createSim() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated drivetrain on real robot", false);
        return new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(MODULE_CONFIG),
                new ModuleIOSim(MODULE_CONFIG),
                new ModuleIOSim(MODULE_CONFIG),
                new ModuleIOSim(MODULE_CONFIG)
        );
    }

    public static Drivetrain createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy drivetrain on real robot", false);
        return new Drivetrain(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {}
        );
    }
}
