package org.team1540.swervedrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
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

    private AimingParameters latestSpeakerParameters = null;
    private AimingParameters latestPassingParameters = null;

    private boolean driveSimConfigured = false;
    private SwerveDriveSimulation driveSim;

    private final Field2d field = new Field2d();

    private RobotState() {
        SmartDashboard.putData(field);
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
        return Rotation2d.fromDegrees(ANGLE_COEFF * Math.pow(distanceMeters, ANGLE_EXP));
    }

    @AutoLogOutput(key = "Aiming/Speaker/Parameters")
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
        return latestSpeakerParameters;
    }

    public void configureDriveSim(SwerveDriveSimulation driveSim) {
        if (!driveSimConfigured) {
            if (!Robot.isSimulation()) {
                throw new IllegalStateException("Cannot configure drive simulation when not in simulation");
            }
            this.driveSim = driveSim;
            driveSimConfigured = true;
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);
        }
    }

    public Pose2d getSimulatedRobotPose() {
        if (!driveSimConfigured) return Pose2d.kZero;
        return driveSim.getSimulatedDriveTrainPose();
    }
}
