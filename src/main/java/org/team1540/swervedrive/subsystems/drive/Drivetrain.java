package org.team1540.swervedrive.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.team1540.swervedrive.util.math.EqualsUtil.*;
import static org.team1540.swervedrive.util.math.GeomUtil.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.Robot;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.CharacterizationCommands;
import org.team1540.swervedrive.generated.TunerConstants;
import org.team1540.swervedrive.util.*;
import org.team1540.swervedrive.util.swerve.ModuleLimits;
import org.team1540.swervedrive.util.swerve.SwerveSetpointGenerator;
import org.team1540.swervedrive.util.swerve.SwerveSetpointGenerator.SwerveSetpoint;

public class Drivetrain extends SubsystemBase {
    public static final double ODOMETRY_FREQUENCY = 250.0;

    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontRight.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
    public static final double MAX_LINEAR_SPEED_MPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_STEER_SPEED_RADS_PER_SEC =
            DCMotor.getFalcon500(1).withReduction(TunerConstants.FrontLeft.SteerMotorGearRatio).freeSpeedRadPerSec;
    public static final double MAX_ANGULAR_SPEED_RADS_PER_SEC = MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;

    public static final ModuleLimits MODULE_LIMITS =
            new ModuleLimits(MAX_LINEAR_SPEED_MPS, Units.feetToMeters(75.0), MAX_STEER_SPEED_RADS_PER_SEC);

    public enum DriveMode {
        /** Standard drive mode, driving according to desired chassis speeds */
        DEFAULT,

        /** Characterizing drive motor velocity feedforwards */
        FF_CHARACTERIZATION,

        /** Spinning in a circle and using gyro rotation to characterize wheel radius */
        WHEEL_RADIUS_CHARACTERIZATION
    }

    private static boolean hasInstance;

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private Rotation2d fieldOrientationOffset = Rotation2d.kZero;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4]; // For odometry delta filtering
    private double lastOdometryUpdateTime = 0.0;

    @AutoLogOutput(key = "Drivetrain/CurrentDriveMode")
    private DriveMode currentDriveMode = DriveMode.DEFAULT;

    private double characterizationInput;
    private boolean forceModuleRotation = false;

    @AutoLogOutput(key = "Drivetrain/DesiredSpeeds")
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    });

    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(kinematics);

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

        AutoBuilder.configure(
                RobotState.getInstance()::getRobotPose,
                RobotState.getInstance()::resetPose,
                RobotState.getInstance()::getRobotVelocity,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                new RobotConfig(
                        Constants.ROBOT_MASS_KG,
                        Constants.ROBOT_MOI_KG_M2,
                        new ModuleConfig(
                                TunerConstants.FrontLeft.WheelRadius,
                                MAX_LINEAR_SPEED_MPS,
                                Constants.WHEEL_COF,
                                DCMotor.getKrakenX60(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                                TunerConstants.FrontLeft.SlipCurrent,
                                1),
                        getModuleTranslations()),
                AllianceFlipUtil::shouldFlip,
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> RobotState.getInstance().setActiveTrajectory(activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(RobotState.getInstance()::setCurrentTrajectoryTarget);

        for (int i = 0; i < 4; i++) {
            lastModulePositions[i] = modules[i].getPosition();
        }
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (Module module : modules) module.periodic();
        odometryLock.unlock();

        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

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
                if (Math.abs(velocity) > MAX_LINEAR_SPEED_MPS * 5
                        || Math.abs(turnVelocity) > MAX_STEER_SPEED_RADS_PER_SEC * 5) {
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
            }
        }

        // Update robot velocities
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        speeds.omegaRadiansPerSecond =
                gyroInputs.connected ? gyroInputs.yawVelocityRadPerSec : speeds.omegaRadiansPerSecond;
        RobotState.getInstance().addVelocityData(speeds);

        SwerveModuleState[] setpointStates = new SwerveModuleState[4];

        // Run modules based on current drive mode
        switch (currentDriveMode) {
            case FF_CHARACTERIZATION -> {
                for (Module module : modules) module.runCharacterization(characterizationInput);
            }
            case WHEEL_RADIUS_CHARACTERIZATION -> setpointStates =
                    kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, characterizationInput));
            default -> {
                Logger.recordOutput(
                        "Drivetrain/SwerveStates/DesiredSetpoints", kinematics.toSwerveModuleStates(desiredSpeeds));
                // Generate kinematically feasible setpoint
                currentSetpoint = setpointGenerator.generateSetpoint(
                        MODULE_LIMITS, currentSetpoint, desiredSpeeds, forceModuleRotation, Constants.LOOP_PERIOD_SECS);
                setpointStates = currentSetpoint.moduleStates();
            }
        }
        if (currentDriveMode != DriveMode.FF_CHARACTERIZATION) {
            for (int i = 0; i < 4; i++) {
                modules[i].runSetpoint(setpointStates[i]);
            }
            Logger.recordOutput("Drivetrain/SwerveStates/OptimizedSetpoints", setpointStates);
        }

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
        speeds.discretize(Constants.LOOP_PERIOD_SECS);
        desiredSpeeds = speeds;
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
        for (int i = 0; i < 4; i++) headings[i] = getModuleTranslations()[i].getAngle();
        kinematics.resetHeadings(headings);
        forceModuleRotation = true;
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

    /** Orients all modules forward and applies the specified voltage to the drive motors */
    private void runFFCharacterization(double volts) {
        characterizationInput = volts;
        currentDriveMode = DriveMode.FF_CHARACTERIZATION;
    }

    /**
     * Spins the robot in a circle at the specified angular velocity to characterize wheel radius
     */
    private void runWheelRadiusCharacterization(double omega) {
        characterizationInput = omega;
        currentDriveMode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
    }

    /** Ends characterization and returns to default drive behavior */
    private void endCharacterization() {
        characterizationInput = 0;
        currentDriveMode = DriveMode.DEFAULT;
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

    /** Returns an array of module translations. */
    public Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY),
        };
    }

    /** Returns a command that drives the robot based on joystick inputs */
    public Command teleopDriveCommand(
            DoubleSupplier controllerX,
            DoubleSupplier controllerY,
            DoubleSupplier controllerOmega,
            BooleanSupplier fieldRelative) {
        return Commands.run(
                        () -> {
                            double xPercent = controllerX.getAsDouble();
                            double yPercent = controllerY.getAsDouble();
                            double omega = JoystickUtil.smartDeadzone(controllerOmega.getAsDouble(), 0.1);

                            double linearMagnitude = JoystickUtil.smartDeadzone(Math.hypot(xPercent, yPercent), 0.1);
                            Rotation2d linearDirection = new Rotation2d(xPercent, yPercent);
                            Translation2d linearVelocity = new Pose2d(Translation2d.kZero, linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                                    .getTranslation();
                            var speeds = new ChassisSpeeds(
                                    linearVelocity.getX() * MAX_LINEAR_SPEED_MPS,
                                    linearVelocity.getY() * MAX_LINEAR_SPEED_MPS,
                                    omega * MAX_ANGULAR_SPEED_RADS_PER_SEC);
                            if (fieldRelative.getAsBoolean()) {
                                speeds.toRobotRelativeSpeeds(rawGyroRotation.minus(fieldOrientationOffset));
                            }
                            runVelocity(speeds);
                        },
                        this)
                .finallyDo(this::stop);
    }

    public Command feedforwardCharacterization() {
        return CharacterizationCommands.feedforward(
                        this::runFFCharacterization, this::getFFCharacterizationVelocity, this)
                .finallyDo(this::endCharacterization);
    }

    public Command wheelRadiusCharacterization() {
        return CharacterizationCommands.wheelRadius(
                        this::runWheelRadiusCharacterization,
                        () -> rawGyroRotation.getRadians(),
                        this::getWheelRadiusCharacterizationPositions,
                        this)
                .finallyDo(this::endCharacterization);
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
        return new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
    }

    public static Drivetrain createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy drivetrain on real robot", false);
        return new Drivetrain(
                new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    }
}
