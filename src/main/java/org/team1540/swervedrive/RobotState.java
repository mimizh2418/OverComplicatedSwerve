package org.team1540.swervedrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private boolean poseEstimatorConfigured = false;
    private SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

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
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, double timestamp) {
        lastModulePositions = modulePositions;
        lastGyroRotation = gyroAngle;
        if (poseEstimatorConfigured) {
            poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);
            field.setRobotPose(getRobotPose());
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

    @AutoLogOutput(key = "SimState/SimulatedRobotPose")
    public Pose2d getSimulatedRobotPose() {
        if (!driveSimConfigured) return Pose2d.kZero;
        return driveSim.getSimulatedDriveTrainPose();
    }
}
