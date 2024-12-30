package org.team1540.swervedrive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.subsystems.pivot.Pivot;
import org.team1540.swervedrive.subsystems.turret.Turret;
import org.team1540.swervedrive.subsystems.vision.AprilTagVision;
import org.team1540.swervedrive.util.AllianceFlipUtil;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class RobotState {
    public record AimingParameters(Rotation2d turretAngle, Rotation2d pivotAngle, double effectiveDistanceMeters) {}

    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private static final LoggedTunableNumber lookaheadSeconds =
            new LoggedTunableNumber("Aiming/LookaheadSeconds", 0.15);

    private boolean poseEstimatorConfigured = false;
    private SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private final Timer resetTimer = new Timer();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private Trajectory<SwerveSample> activeTrajectory = null;

    private final Field2d field = new Field2d();

    private Rotation2d turretAngle = Rotation2d.kZero;
    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer = TimeInterpolatableBuffer.createBuffer(0.2);
    private double turretVelocityRPS = 0.0;

    private Rotation2d pivotAngle = Rotation2d.kZero;

    private final InterpolatingDoubleTreeMap pivotSpeakerMapMetersToDegrees = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap pivotPassingMapMetersToDegrees = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterPassingMapMetersToRPM = new InterpolatingDoubleTreeMap();

    private AimingParameters latestSpeakerParameters = null;
    private AimingParameters latestPassingParameters = null;

    private RobotState() {
        SmartDashboard.putData(field);

        pivotSpeakerMapMetersToDegrees.put(1.0, 57.80 + 4.0);
        pivotSpeakerMapMetersToDegrees.put(1.5, 49.00 + 2.5);
        pivotSpeakerMapMetersToDegrees.put(2.0, 42.75);
        pivotSpeakerMapMetersToDegrees.put(2.5, 36.70);
        pivotSpeakerMapMetersToDegrees.put(3.0, 34.00);
        pivotSpeakerMapMetersToDegrees.put(3.5, 31.20);
        pivotSpeakerMapMetersToDegrees.put(4.0, 29.50);
        pivotSpeakerMapMetersToDegrees.put(4.5, 27.50);
        pivotSpeakerMapMetersToDegrees.put(5.0, 26.00);
        pivotSpeakerMapMetersToDegrees.put(5.5, 24.85);
        pivotSpeakerMapMetersToDegrees.put(6.0, 23.60);
        pivotSpeakerMapMetersToDegrees.put(6.5, 22.80);

        pivotPassingMapMetersToDegrees.put(1.0, 55.0 + 1.0);
        pivotPassingMapMetersToDegrees.put(2.0, 55.0 + 1.0);
        pivotPassingMapMetersToDegrees.put(3.0, 55.0 + 1.0);
        pivotPassingMapMetersToDegrees.put(4.0, 55.0 + 1.0);
        pivotPassingMapMetersToDegrees.put(7.0, 55.0 + 1.0);
        pivotPassingMapMetersToDegrees.put(8.0, 55.0 + 1.0);
        pivotPassingMapMetersToDegrees.put(9.5, 50.0 + 1.0);
        pivotPassingMapMetersToDegrees.put(11.5, 45.0 + 1.0);

        shooterPassingMapMetersToRPM.put(1.0, 2000.0 * 0.86);
        shooterPassingMapMetersToRPM.put(2.0, 3000.0 * 0.86);
        shooterPassingMapMetersToRPM.put(3.0, 4500.0 * 0.86);
        shooterPassingMapMetersToRPM.put(4.0, 5200.0 * 0.86);
        shooterPassingMapMetersToRPM.put(7.0, 6000.0 * 0.75);
        shooterPassingMapMetersToRPM.put(8.0, 6800.0 * 0.75);
        shooterPassingMapMetersToRPM.put(9.5, 7200.0 * 0.75);
        shooterPassingMapMetersToRPM.put(11.5, 8000.0 * 0.75);
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
        }
        invalidateAimingCache();
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
            poseEstimator.resetPosition(lastGyroRotation, lastModulePositions, newPose);
            resetTimer.restart();
        }
        if (Constants.currentMode == Constants.Mode.SIM) SimState.getInstance().resetSimPose(newPose);
        invalidateAimingCache();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getRobotPose() {
        if (!poseEstimatorConfigured) return Pose2d.kZero;
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRobotRotation() {
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
        velocity.toFieldRelativeSpeeds(getRobotRotation());
        return velocity;
    }

    public Pose2d predictPose(Pose2d original, double lookaheadSeconds) {
        if (!poseEstimatorConfigured) return Pose2d.kZero;

        ChassisSpeeds velocity = getFieldRelativeVelocity();
        return new Pose2d(
                original.getX() + velocity.vxMetersPerSecond * lookaheadSeconds,
                original.getY() + velocity.vyMetersPerSecond * lookaheadSeconds,
                original.getRotation().plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookaheadSeconds)));
    }

    public void addTurretAngleData(Rotation2d angle, double velocityRPS, double timestampSeconds) {
        turretAngle = angle;
        turretVelocityRPS = velocityRPS;
        turretAngleBuffer.addSample(timestampSeconds, angle);
    }

    public void addTurretPoseObservation(AprilTagVision.PoseObservation observation, Vector<N3> stdDevs) {
        if (poseEstimatorConfigured && resetTimer.hasElapsed(0.25)) {
            Optional<Rotation2d> interpolatedTurretAngle = turretAngleBuffer.getSample(observation.timestampSecs());
            Rotation2d turretAngle = interpolatedTurretAngle.orElse(this.turretAngle);

            Pose2d pose = observation
                    .pose()
                    .toPose2d()
                    .transformBy(new Transform2d(Turret.ORIGIN_METERS.toTranslation2d(), turretAngle).inverse());

            poseEstimator.setVisionMeasurementStdDevs(stdDevs);
            poseEstimator.addVisionMeasurement(pose, observation.timestampSecs());
        }
        invalidateAimingCache();
    }

    public Rotation2d getTurretAngle() {
        return turretAngle;
    }

    public double getTurretVelocityRPS() {
        return turretVelocityRPS;
    }

    public Pose2d getTurretPose() {
        return getRobotPose().transformBy(new Transform2d(Pivot.TURRET_TO_PIVOT_METERS.toTranslation2d(), turretAngle));
    }

    public void addPivotAngleData(Rotation2d angle) {
        pivotAngle = angle;
    }

    public Rotation2d getPivotAngle() {
        return pivotAngle;
    }

    public void updateMechanismVisualization() {
        Pose3d turretPose = new Pose3d(Turret.ORIGIN_METERS, new Rotation3d(turretAngle));
        Pose3d pivotPose = turretPose.transformBy(
                new Transform3d(Pivot.TURRET_TO_PIVOT_METERS, new Rotation3d(0.0, -pivotAngle.getRadians(), 0.0)));

        Logger.recordOutput("Turret/Mechanism3d", turretPose);
        Logger.recordOutput("Pivot/Mechanism3d", pivotPose);
    }

    public AimingParameters getSpeakerAimingParameters() {
        if (latestSpeakerParameters != null) return latestSpeakerParameters;

        Pose2d predictedTurretPose = predictPose(getTurretPose(), lookaheadSeconds.get());
        Translation2d turretToTargetTranslation =
                FieldConstants.getSpeakerPose().getTranslation().minus(predictedTurretPose.getTranslation());
        Rotation2d turretAngle = turretToTargetTranslation.getAngle().minus(getRobotRotation());
        double effectiveDistanceMeters = turretToTargetTranslation.getNorm();

        Logger.recordOutput("Aiming/Speaker/PredictedPose", predictPose(getRobotPose(), lookaheadSeconds.get()));

        latestSpeakerParameters = new AimingParameters(
                turretAngle,
                Rotation2d.fromDegrees(pivotSpeakerMapMetersToDegrees.get(effectiveDistanceMeters)),
                effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Speaker/Parameters", latestSpeakerParameters);
        return latestSpeakerParameters;
    }

    public AimingParameters getPassingAimingParameters() {
        if (latestPassingParameters != null) return latestPassingParameters;

        Pose2d predictedTurretPose = predictPose(getTurretPose(), lookaheadSeconds.get());
        Pose2d predictedRobotPose = predictPose(getRobotPose(), lookaheadSeconds.get());
        Translation2d turretToTargetTranslation =
                FieldConstants.getPassingTarget().getTranslation().minus(predictedTurretPose.getTranslation());
        Rotation2d turretAngle = turretToTargetTranslation.getAngle().minus(predictedRobotPose.getRotation());
        double effectiveDistanceMeters = turretToTargetTranslation.getNorm();

        Logger.recordOutput("Aiming/Passing/PredictedPose", predictPose(getRobotPose(), lookaheadSeconds.get()));

        latestPassingParameters = new AimingParameters(
                turretAngle,
                Rotation2d.fromDegrees(pivotPassingMapMetersToDegrees.get(effectiveDistanceMeters)),
                effectiveDistanceMeters);
        Logger.recordOutput("Aiming/Passing/Parameters", latestPassingParameters);
        return latestPassingParameters;
    }

    public double getPassingShooterRPM() {
        return shooterPassingMapMetersToRPM.get(getPassingAimingParameters().effectiveDistanceMeters());
    }

    private void invalidateAimingCache() {
        latestSpeakerParameters = null;
        latestPassingParameters = null;
    }
}
