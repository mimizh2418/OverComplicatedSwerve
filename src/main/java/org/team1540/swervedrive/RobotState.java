package org.team1540.swervedrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLogOutput;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private Twist2d robotVelocity = new Twist2d();

    private SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private Rotation2d rawGyroRotation = new Rotation2d();

    private RobotState() {
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        Drivetrain.KINEMATICS,
                        new Rotation2d(),
                        new SwerveModulePosition[]{
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition()},
                        new Pose2d(),
                        VecBuilder.fill(0.1, 0.1, 0.1),
                        VecBuilder.fill(0.5, 0.5, 5.0));
    }

    public void addOdometryObservation(
            SwerveModulePosition[] modulePositions,
            Rotation2d gyroAngle,
            double timestamp) {
        if (gyroAngle != null) rawGyroRotation = gyroAngle;
        else {
            Twist2d twist = Drivetrain.KINEMATICS.toTwist2d(
                    new SwerveDriveWheelPositions(lastModulePositions),
                    new SwerveDriveWheelPositions(modulePositions));
            rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
        }
        lastModulePositions = modulePositions;

        poseEstimator.updateWithTime(timestamp, rawGyroRotation, modulePositions);
    }

    public void addVelocityData(Twist2d velocity) {
        robotVelocity = velocity;
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, newPose);
    }

    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getRobotPose().getRotation();
    }

    public Rotation2d getRawGyroRotation() {
        return rawGyroRotation;
    }

    @AutoLogOutput(key = "RobotState/RobotVelocity")
    public Twist2d getRobotVelocity() {
        return robotVelocity;
    }

    @AutoLogOutput(key = "RobotState/FieldRelativeVelocity")
    public Twist2d getFieldRelativeVelocity() {
        Translation2d linearFieldVelocity =
                new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(getRotation());
        return new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
    }
}
