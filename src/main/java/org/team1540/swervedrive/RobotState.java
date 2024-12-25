package org.team1540.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.subsystems.arm.Arm;
import org.team1540.swervedrive.subsystems.vision.AprilTagVision;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class RobotState {
    public record AimingParameters(Rotation2d driveHeading, Rotation2d armAngle, double effectiveDistanceMeters) {}

    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private static final LoggedTunableNumber aimingLookaheadSeconds =
            new LoggedTunableNumber("Aiming/lookaheadSeconds", 0.1);

    private boolean poseEstimatorConfigured = false;
    private SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private final InterpolatingDoubleTreeMap passingArmAngleInterpolator = new InterpolatingDoubleTreeMap();

    private AimingParameters latestSpeakerParameters = null;
    private AimingParameters latestPassingParameters = null;

    private Rotation2d currentArmAngle = Arm.MIN_ANGLE;
    private Rotation2d goalArmAngle = Arm.MIN_ANGLE;

    private double shooterLeftVelocityRPM = 0.0;
    private double shooterRightVelocityRPM = 0.0;

    private boolean driveSimConfigured = false;
    private SwerveDriveSimulation driveSim;

    private final Field2d field = new Field2d();

    private RobotState() {
        SmartDashboard.putData(field);

        passingArmAngleInterpolator.put(Units.feetToMeters(33.52713263758169), 43.0);
        passingArmAngleInterpolator.put(Units.feetToMeters(28.31299227120627), 47.0);
        passingArmAngleInterpolator.put(Units.feetToMeters(25.587026383435525), 56.0);
    }

    public void configurePoseEstimation(SwerveDriveKinematics kinematics) {
        if (!poseEstimatorConfigured) {
            poseEstimator = new SwerveDrivePoseEstimator(
                    kinematics,
                    lastGyroRotation,
                    lastModulePositions,
                    Pose2d.kZero,
                    VecBuilder.fill(0.1, 0.1, 0.1),
                    VecBuilder.fill(0.5, 0.5, 5.0));
            poseEstimatorConfigured = true;
        }
        invalidateAimingCache();
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, double timestamp) {
        lastModulePositions = modulePositions;
        lastGyroRotation = gyroAngle;
        if (poseEstimatorConfigured) {
            poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);
            field.setRobotPose(getRobotPose());
            invalidateAimingCache();
        }
    }

    public void addVisionObservation(AprilTagVision.PoseObservation observation, Vector<N3> stdDevs) {
        if (poseEstimatorConfigured) {
            poseEstimator.setVisionMeasurementStdDevs(stdDevs);
            poseEstimator.addVisionMeasurement(observation.pose().toPose2d(), observation.timestampSecs());
            invalidateAimingCache();
        }
    }

    public void addVelocityData(ChassisSpeeds velocity) {
        robotVelocity = velocity;
    }

    public void setActiveTrajectory(Pose2d[] trajectory) {
        field.getObject("trajectory").setPoses(trajectory);
        Logger.recordOutput("Odometry/Trajectory/ActiveTrajectory", trajectory);
    }

    public void setCurrentTrajectoryTarget(Pose2d target) {
        Logger.recordOutput("Odometry/Trajectory/CurrentTrajectoryTarget", target);
    }

    public void resetPose(Pose2d newPose) {
        if (poseEstimatorConfigured) poseEstimator.resetPosition(lastGyroRotation, lastModulePositions, newPose);
        if (driveSimConfigured) driveSim.setSimulationWorldPose(newPose);
        invalidateAimingCache();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getRobotPose() {
        if (!poseEstimatorConfigured) return Pose2d.kZero;
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        if (!poseEstimatorConfigured) return Rotation2d.kZero;
        return getRobotPose().getRotation();
    }

    @AutoLogOutput(key = "Odometry/RobotVelocity")
    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    @AutoLogOutput(key = "Odometry/FieldRelativeVelocity")
    public ChassisSpeeds getFieldRelativeVelocity() {
        var rotated = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                .rotateBy(getRotation());
        return new ChassisSpeeds(rotated.getX(), rotated.getY(), robotVelocity.omegaRadiansPerSecond);
    }

    public Pose2d predictRobotPose(double lookaheadSeconds) {
        if (!poseEstimatorConfigured) return Pose2d.kZero;
        ChassisSpeeds velocity = getFieldRelativeVelocity();
        Pose2d pose = getRobotPose();
        return new Pose2d(
                pose.getX() + velocity.vxMetersPerSecond * lookaheadSeconds,
                pose.getY() + velocity.vyMetersPerSecond * lookaheadSeconds,
                pose.getRotation().plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookaheadSeconds)));
    }

    private void invalidateAimingCache() {
        latestSpeakerParameters = null;
        latestPassingParameters = null;
    }

    private static final double ANGLE_COEFF = 57.254371165197;
    private static final double ANGLE_EXP = -0.593140189605718;

    private Rotation2d calculateSpeakerArmAngle(double distanceMeters) {
        return Rotation2d.fromDegrees(MathUtil.clamp(
                ANGLE_COEFF * Math.pow(distanceMeters, ANGLE_EXP),
                Arm.MIN_ANGLE.getDegrees(),
                Arm.ArmState.SUBWOOFER.angleSupplier.get().getDegrees()));
    }

    public AimingParameters getSpeakerAimingParameters() {
        if (latestSpeakerParameters != null) return latestSpeakerParameters;

        Pose2d predictedPose = predictRobotPose(aimingLookaheadSeconds.get());
        Translation2d robotToTargetTranslation =
                FieldConstants.getSpeakerPose().getTranslation().minus(predictedPose.getTranslation());
        Rotation2d driveHeading = robotToTargetTranslation.getAngle();
        double effectiveDistanceMeters = robotToTargetTranslation.getNorm();

        Logger.recordOutput("Aiming/Speaker/PredictedPose", predictedPose);
        Logger.recordOutput("Aiming/Speaker/EffectiveDistanceMeters", effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Speaker/GoalPose", new Pose2d(predictedPose.getTranslation(), driveHeading));

        latestSpeakerParameters = new AimingParameters(
                driveHeading, calculateSpeakerArmAngle(effectiveDistanceMeters), effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Speaker/Parameters", latestSpeakerParameters);
        return latestSpeakerParameters;
    }

    private Rotation2d calculatePassingArmAngle(double distanceMeters) {
        return Rotation2d.fromDegrees(passingArmAngleInterpolator.get(distanceMeters));
    }

    public AimingParameters getPassingAimingParameters() {
        if (latestPassingParameters != null) return latestPassingParameters;

        Pose2d predictedPose = predictRobotPose(aimingLookaheadSeconds.get());
        Translation2d robotToTargetTranslation =
                FieldConstants.getPassingTarget().getTranslation().minus(predictedPose.getTranslation());
        Rotation2d driveHeading = robotToTargetTranslation.getAngle();
        double effectiveDistanceMeters = robotToTargetTranslation.getNorm();

        Logger.recordOutput("Aiming/Passing/PredictedPose", predictedPose);
        Logger.recordOutput("Aiming/Passing/EffectiveDistanceMeters", effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Passing/GoalPose", new Pose2d(predictedPose.getTranslation(), driveHeading));

        latestPassingParameters = new AimingParameters(
                driveHeading, calculatePassingArmAngle(effectiveDistanceMeters), effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Passing/Parameters", latestPassingParameters);
        return latestPassingParameters;
    }

    public AimingParameters getLowPassingAimingParameters() {
        AimingParameters passParams = getPassingAimingParameters();
        var lowPassParams = new AimingParameters(
                passParams.driveHeading(), Arm.ArmState.STOW.angleSupplier.get(), passParams.effectiveDistanceMeters());
        Logger.recordOutput("Aiming/LowPassing/Parameters", lowPassParams);
        return lowPassParams;
    }

    public void addArmAngleData(Rotation2d armAngle, Rotation2d armGoalAngle) {
        currentArmAngle = armAngle;
        goalArmAngle = armGoalAngle;
    }

    public void addShooterVelocityData(double leftVelocityRPM, double rightVelocityRPM) {
        this.shooterLeftVelocityRPM = leftVelocityRPM;
        this.shooterRightVelocityRPM = rightVelocityRPM;
    }

    public void updateMechanismVisualization() {
        Pose3d currentPose = new Pose3d(Arm.PIVOT_ORIGIN, new Rotation3d(0.0, -currentArmAngle.getRadians(), 0.0));
        Pose3d goalPose = new Pose3d(Arm.PIVOT_ORIGIN, new Rotation3d(0.0, -goalArmAngle.getRadians(), 0.0));

        Logger.recordOutput("Arm/Mechanism3d/Measured", currentPose);
        Logger.recordOutput("Arm/Mechanism3d/Goal", goalPose);
    }

    public void updateSimState() {
        if (Constants.currentMode != Constants.Mode.SIM || !driveSimConfigured) return;
        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("SimState/SimulatedRobotPose", getSimulatedRobotPose());
        Logger.recordOutput(
                "SimState/Notes",
                SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
    }

    public void configureDriveSim(SwerveDriveSimulation driveSim) {
        if (!driveSimConfigured) {
            if (Constants.currentMode != Constants.Mode.SIM) {
                throw new IllegalStateException("Cannot configure drive simulation when not in simulation");
            }
            this.driveSim = driveSim;
            driveSimConfigured = true;
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        }
    }

    public Optional<SwerveDriveSimulation> getDriveSim() {
        if (!driveSimConfigured) return Optional.empty();
        return Optional.of(driveSim);
    }

    public Pose2d getSimulatedRobotPose() {
        if (!driveSimConfigured) return Pose2d.kZero;
        return driveSim.getSimulatedDriveTrainPose();
    }

    private static final double NOTE_VELOCITY_COEFF_MPS_PER_RPM = 17.5 / 6400;

    public void simShootNote() {
        if (!driveSimConfigured) return;
        Pose3d robotToShooter = new Pose3d(
                new Translation3d(Arm.LENGTH_METERS, 0.0, 0.0)
                        .rotateBy(new Rotation3d(0.0, -currentArmAngle.getRadians(), 0.0))
                        .plus(Arm.PIVOT_ORIGIN),
                new Rotation3d(0.0, -currentArmAngle.getRadians(), 0.0));

        GamePieceProjectile note = new NoteOnFly(
                        getSimulatedRobotPose().getTranslation(),
                        robotToShooter.getTranslation().toTranslation2d(),
                        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative()
                                .div(4.0),
                        getSimulatedRobotPose()
                                .getRotation()
                                .plus(robotToShooter.getRotation().toRotation2d()),
                        robotToShooter.getZ(),
                        (shooterLeftVelocityRPM + shooterRightVelocityRPM) / 2 * NOTE_VELOCITY_COEFF_MPS_PER_RPM,
                        currentArmAngle.getRadians())
                .enableBecomeNoteOnFieldAfterTouchGround()
                .withTouchGroundHeight(0.1);
        SimulatedArena.getInstance().addGamePieceProjectile(note);
    }

    public void simAmpNote() {
        if (!driveSimConfigured) return;
        Pose3d robotToAmp = new Pose3d(
                new Translation3d(Arm.LENGTH_METERS, 0.0, 0.0)
                        .rotateBy(new Rotation3d(0.0, -currentArmAngle.getRadians() - Math.toRadians(20.0), 0.0))
                        .plus(Arm.PIVOT_ORIGIN),
                new Rotation3d(0.0, -currentArmAngle.getRadians() - Math.toRadians(135), 0.0));
        GamePieceProjectile note = new NoteOnFly(
                        getSimulatedRobotPose().getTranslation(),
                        robotToAmp.getTranslation().toTranslation2d(),
                        driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative()
                                .div(4.0),
                        getSimulatedRobotPose().getRotation(),
                        robotToAmp.getZ(),
                        1.0,
                        currentArmAngle.getRadians() + Math.toRadians(150))
                .enableBecomeNoteOnFieldAfterTouchGround()
                .withTouchGroundHeight(0.1);
        SimulatedArena.getInstance().addGamePieceProjectile(note);
    }

    public void simEjectNote() {
        if (!driveSimConfigured) return;
        SimulatedArena.getInstance()
                .addGamePiece(new CrescendoNoteOnField(getSimulatedRobotPose()
                        .transformBy(
                                new Transform2d(-(Constants.BUMPER_LENGTH_X_METERS / 2) - 0.05, 0.0, Rotation2d.kZero))
                        .getTranslation()));
    }
}
