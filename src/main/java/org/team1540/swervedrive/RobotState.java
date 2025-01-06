package org.team1540.swervedrive;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.subsystems.vision.AprilTagVision;
import org.team1540.swervedrive.util.AllianceFlipUtil;

public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private boolean poseEstimatorConfigured = false;
    private SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private final Timer resetTimer = new Timer();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private Trajectory<SwerveSample> activeTrajectory = null;

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
            resetTimer.restart();
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

    public void addVisionObservation(AprilTagVision.PoseObservation observation, Vector<N3> stdDevs) {
        if (poseEstimatorConfigured && resetTimer.hasElapsed(0.25)) {
            poseEstimator.setVisionMeasurementStdDevs(stdDevs);
            poseEstimator.addVisionMeasurement(observation.pose().toPose2d(), observation.timestampSecs());
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
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getRotation());
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
}
