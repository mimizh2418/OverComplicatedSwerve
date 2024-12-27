package org.team1540.swervedrive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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
import org.team1540.swervedrive.util.AllianceFlipUtil;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class RobotState {
    public record AimingParameters(
            Rotation2d driveHeading,
            double driveVelocityFFRadPerSec,
            Rotation2d armAngle,
            ShooterSpeeds shooterSpeeds,
            double effectiveDistanceMeters) {}

    public record ShooterSpeeds(double leftRPM, double rightRPM) {
        public ShooterSpeeds plus(double delta) {
            return new ShooterSpeeds(leftRPM + delta, rightRPM + delta);
        }
    }

    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private static final LoggedTunableNumber speakerLookaheadSecs =
            new LoggedTunableNumber("Aiming/Speaker/LookaheadSeconds", 0.15);
    private static final LoggedTunableNumber passingLookaheadSecs =
            new LoggedTunableNumber("Aiming/Passing/LookaheadSeconds", 0.35);
    private static final LoggedTunableNumber rpmCompensationCoeff =
            new LoggedTunableNumber("Aiming/RPMCompensation/Coeff", 0.3);

    private static final double SPEAKER_ANGLE_COEFF = 57.254371165197;
    private static final double SPEAKER_ANGLE_EXP = -0.593140189605718;

    private static final double NOTE_VELOCITY_COEFF_MPS_PER_RPM = 17.5 / 6400;

    private static final ShooterSpeeds SPEAKER_BASE_SHOOTER_SPEEDS = new ShooterSpeeds(5066.0, 7733.0);
    private static final ShooterSpeeds PASS_BASE_SHOOTER_SPEEDS = new ShooterSpeeds(3500.0, 3500.0);

    private boolean poseEstimatorConfigured = false;
    private SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private final Timer resetTimer = new Timer();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private Trajectory<SwerveSample> activeTrajectory = null;

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
            resetTimer.restart();
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
        if (poseEstimatorConfigured && resetTimer.hasElapsed(0.25)) {
            poseEstimator.setVisionMeasurementStdDevs(stdDevs);
            poseEstimator.addVisionMeasurement(observation.pose().toPose2d(), observation.timestampSecs());
            invalidateAimingCache();
        }
    }

    public void addVelocityData(ChassisSpeeds velocity) {
        robotVelocity = velocity;
    }

    public void setActiveTrajectory(Trajectory<SwerveSample> trajectory) {
        activeTrajectory = AllianceFlipUtil.shouldFlip() ? trajectory.flipped() : trajectory;
        field.getObject("trajectory").setPoses(activeTrajectory.getPoses());
        Logger.recordOutput("Odometry/Trajectory/ActiveTrajectory", activeTrajectory.getPoses());
    }

    public void clearActiveTrajectory() {
        field.getObject("trajectory").setPoses();
        Logger.recordOutput("Odometry/Trajectory/ActiveTrajectory", new Pose2d[0]);
        activeTrajectory = null;
    }

    public void setTrajectoryTarget(Pose2d target) {
        Logger.recordOutput("Odometry/Trajectory/TargetPose", target);
    }

    public void resetPose(Pose2d newPose) {
        if (poseEstimatorConfigured) {
            if (driveSimConfigured) driveSim.setSimulationWorldPose(newPose);
            poseEstimator.resetPosition(lastGyroRotation, lastModulePositions, newPose);
            resetTimer.restart();
        }
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
        ChassisSpeeds velocity = new ChassisSpeeds(
                robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond, robotVelocity.omegaRadiansPerSecond);
        velocity.toFieldRelativeSpeeds(getRotation());
        return velocity;
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

    private double calculateDriveOmegaFFRadPerSec(Pose2d target) {
        Vector<N2> targetTangentVector = target.getTranslation()
                .minus(getRobotPose().getTranslation())
                .rotateBy(AllianceFlipUtil.shouldFlip() ? Rotation2d.kCCW_90deg : Rotation2d.kCW_90deg)
                .toVector();
        Vector<N2> targetTangentVectorNormalized = targetTangentVector.div(targetTangentVector.norm());
        Vector<N2> robotVelocityVector =
                VecBuilder.fill(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond);
        double scalarProjection = robotVelocityVector.dot(targetTangentVectorNormalized);
        double distance =
                target.getTranslation().minus(getRobotPose().getTranslation()).getNorm();
        return -scalarProjection / distance;
    }

    private double calculateRPMCompensation(Pose2d predictedPose, Pose2d target) {
        Vector<N2> robotToTarget =
                target.getTranslation().minus(predictedPose.getTranslation()).toVector();
        Vector<N2> robotVelocity =
                VecBuilder.fill(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond);
        Vector<N2> robotToTargetNormalized = robotToTarget.div(robotToTarget.norm());
        double scalarProjection = robotVelocity.dot(robotToTargetNormalized);
        return rpmCompensationCoeff.get()
                * (scalarProjection / goalArmAngle.getCos())
                / NOTE_VELOCITY_COEFF_MPS_PER_RPM;
    }

    private Rotation2d calculateSpeakerArmAngle(double distanceMeters) {
        return Rotation2d.fromDegrees(MathUtil.clamp(
                SPEAKER_ANGLE_COEFF * Math.pow(distanceMeters, SPEAKER_ANGLE_EXP),
                Arm.MIN_ANGLE.getDegrees(),
                Arm.ArmState.SUBWOOFER.angleSupplier.get().getDegrees()));
    }

    public AimingParameters getSpeakerAimingParameters() {
        if (latestSpeakerParameters != null) return latestSpeakerParameters;

        Pose2d predictedPose = predictRobotPose(speakerLookaheadSecs.get());
        Translation2d robotToTargetTranslation =
                FieldConstants.getSpeakerPose().getTranslation().minus(predictedPose.getTranslation());
        Rotation2d driveHeading = robotToTargetTranslation.getAngle();
        double effectiveDistanceMeters = robotToTargetTranslation.getNorm();
        double rpmCompensation = calculateRPMCompensation(predictedPose, FieldConstants.getSpeakerPose());

        Logger.recordOutput("Aiming/Speaker/PredictedPose", predictedPose);
        Logger.recordOutput("Aiming/Speaker/EffectiveDistanceMeters", effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Speaker/GoalPose", new Pose2d(predictedPose.getTranslation(), driveHeading));
        Logger.recordOutput("Aiming/Speaker/RPMCompensation", rpmCompensation);

        latestSpeakerParameters = new AimingParameters(
                driveHeading,
                calculateDriveOmegaFFRadPerSec(FieldConstants.getSpeakerPose()),
                calculateSpeakerArmAngle(effectiveDistanceMeters),
                SPEAKER_BASE_SHOOTER_SPEEDS.plus(rpmCompensation),
                effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Speaker/Parameters", latestSpeakerParameters);
        return latestSpeakerParameters;
    }

    private Rotation2d calculatePassingArmAngle(double distanceMeters) {
        return Rotation2d.fromDegrees(
                MathUtil.clamp(passingArmAngleInterpolator.get(distanceMeters), Arm.MIN_ANGLE.getDegrees(), 90.0));
    }

    public AimingParameters getPassingAimingParameters() {
        if (latestPassingParameters != null) return latestPassingParameters;

        Pose2d predictedPose = predictRobotPose(passingLookaheadSecs.get());
        Translation2d robotToTargetTranslation =
                FieldConstants.getPassingTarget().getTranslation().minus(predictedPose.getTranslation());
        Rotation2d driveHeading = robotToTargetTranslation.getAngle();
        double effectiveDistanceMeters = robotToTargetTranslation.getNorm();
        double rpmCompensation = calculateRPMCompensation(predictedPose, FieldConstants.getPassingTarget());

        Logger.recordOutput("Aiming/Passing/PredictedPose", predictedPose);
        Logger.recordOutput("Aiming/Passing/EffectiveDistanceMeters", effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Passing/GoalPose", new Pose2d(predictedPose.getTranslation(), driveHeading));

        latestPassingParameters = new AimingParameters(
                driveHeading,
                calculateDriveOmegaFFRadPerSec(FieldConstants.getPassingTarget()),
                calculatePassingArmAngle(effectiveDistanceMeters),
                PASS_BASE_SHOOTER_SPEEDS.plus(rpmCompensation),
                effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Passing/Parameters", latestPassingParameters);
        return latestPassingParameters;
    }

    public AimingParameters getLowPassingAimingParameters() {
        AimingParameters passParams = getPassingAimingParameters();
        var lowPassParams = new AimingParameters(
                passParams.driveHeading(),
                passParams.driveVelocityFFRadPerSec(),
                Arm.ArmState.STOW.angleSupplier.get(),
                passParams.shooterSpeeds(),
                passParams.effectiveDistanceMeters());
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
        Logger.recordOutput("SimState/SimulatedRobotPose", getSimulatedRobotPose());
        Logger.recordOutput(
                "SimState/Notes",
                SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));

        SimulatedArena.getInstance().simulationPeriodic();
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
                                .div(2.0),
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
                                .div(2.0),
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
