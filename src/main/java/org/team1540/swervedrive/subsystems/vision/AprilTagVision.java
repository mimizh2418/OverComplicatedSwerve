package org.team1540.swervedrive.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;

public class AprilTagVision extends SubsystemBase {
    public record TagObservation(int tagId, Transform3d robotToTag, Pose3d fieldToTag) {}

    public record PoseObservation(
            double timestampSecs,
            Pose3d pose,
            int numTags,
            double avgTagDistanceMeters,
            double ambiguity,
            PoseObservationSource source) {}

    public enum PoseObservationSource {
        MEGATAG_1(VecBuilder.fill(0.1, 0.1, 0.5)),
        MEGATAG_2(VecBuilder.fill(
                0.05, 0.05, Double.POSITIVE_INFINITY)), // Megatag2 does not provide useful rotation data
        PHOTONVISION(VecBuilder.fill(0.08, 0.08, 0.4));

        public final Vector<N3> stdDevBaseline;

        PoseObservationSource(Vector<N3> stdDevBaseline) {
            this.stdDevBaseline = stdDevBaseline;
        }
    }

    public record CameraConfig(String name, Transform3d robotToCamera) {}

    private static final CameraConfig[] cameraConfigs = new CameraConfig[] {
        new CameraConfig(
                "front-left",
                new Transform3d(
                        Units.inchesToMeters(12.797),
                        Units.inchesToMeters(12.290),
                        Units.inchesToMeters(8.0),
                        new Rotation3d(0.0, Math.toRadians(-30), 0.0) // Pitch up 30 degrees
                                .rotateBy(new Rotation3d(0.0, 0.0, Math.toRadians(45))))), // Yaw left 45 degrees
        new CameraConfig(
                "front-right",
                new Transform3d(
                        Units.inchesToMeters(12.797),
                        Units.inchesToMeters(-12.290),
                        Units.inchesToMeters(8.0),
                        new Rotation3d(0.0, Math.toRadians(-30), 0.0) // Pitch up 30 degrees
                                .rotateBy(new Rotation3d(0.0, 0.0, Math.toRadians(-45))))), // Yaw right 45 degrees
        new CameraConfig(
                "back-left",
                new Transform3d(
                        Units.inchesToMeters(-12.797),
                        Units.inchesToMeters(12.290),
                        Units.inchesToMeters(8.0),
                        new Rotation3d(0.0, Math.toRadians(-30), 0.0) // Pitch up 30 degrees
                                .rotateBy(new Rotation3d(0.0, 0.0, Math.toRadians(135))))), // Yaw left 135 degrees
        new CameraConfig(
                "back-right",
                new Transform3d(
                        Units.inchesToMeters(-12.797),
                        Units.inchesToMeters(-12.290),
                        Units.inchesToMeters(8.0),
                        new Rotation3d(0.0, Math.toRadians(-30), 0.0) // Pitch up 30 degrees
                                .rotateBy(new Rotation3d(0.0, 0.0, Math.toRadians(-135))))), // Yaw right 135 degrees
    };

    // Pose filtering parameters
    private static final double MAX_AMBIGUITY = 0.3;
    private static final double MAX_AVG_TAG_DISTANCE_METERS = 8.0;
    // If you're more than 0.75 meters above the ground you probably have more pressing issues than bad vision data
    // (unless you want to use vision while climbing???)
    private static final double MAX_Z_ERROR_METERS = 0.75;

    private static final boolean hasInstance = false;

    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputsAutoLogged[] inputs;

    private final Alert[] disconnectedAlerts;

    @SuppressWarnings("resource")
    private AprilTagVision(AprilTagVisionIO... io) {
        if (hasInstance) throw new IllegalStateException("Instance of AprilTagVision already exists");
        if (io.length > cameraConfigs.length)
            throw new IllegalArgumentException("Too many cameras passed to AprilTagVision, only " + cameraConfigs.length
                    + " camera configs exist");

        this.io = io;
        this.inputs = new AprilTagVisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new AprilTagVisionIOInputsAutoLogged();
        }
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < io.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("AprilTag camera " + cameraConfigs[i].name + " is disconnected", Alert.AlertType.kError);
        }
    }

    public void periodic() {
        ArrayList<Transform3d> robotToTags = new ArrayList<>();
        ArrayList<Pose3d> tagPoses = new ArrayList<>();
        ArrayList<Pose3d> acceptedRobotPoses = new ArrayList<>();
        ArrayList<Pose3d> rejectedRobotPoses = new ArrayList<>();
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + cameraConfigs[i].name(), inputs[i]);

            // Process tag observations
            for (TagObservation observation : inputs[i].latestTargetObservations) {
                robotToTags.add(observation.robotToTag);
                tagPoses.add(observation.fieldToTag);
            }

            // Filter and process pose estimates
            for (PoseObservation observation : inputs[i].poseObservations) {
                boolean acceptPose = shouldAcceptPose(observation);
                if (acceptPose) acceptedRobotPoses.add(observation.pose);
                else {
                    rejectedRobotPoses.add(observation.pose);
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor =
                        (observation.avgTagDistanceMeters * observation.avgTagDistanceMeters) / observation.numTags;
                Vector<N3> stdDevs = observation.source.stdDevBaseline.times(stdDevFactor);
                RobotState.getInstance().addVisionObservation(observation, stdDevs);
            }

            disconnectedAlerts[i].set(!inputs[i].connected);
        }

        // Log data
        Logger.recordOutput("Vision/AcceptedPoses", acceptedRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/RejectedPoses", rejectedRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/TagTransforms", robotToTags.toArray(new Transform3d[0]));
        Logger.recordOutput("Vision/TagPoses", tagPoses.toArray(new Pose3d[0]));
    }

    private boolean shouldAcceptPose(PoseObservation observation) {
        return observation.numTags >= 1
                && !(observation.numTags == 1 && observation.ambiguity > MAX_AMBIGUITY)
                && observation.avgTagDistanceMeters < MAX_AVG_TAG_DISTANCE_METERS
                && Math.abs(observation.pose.getZ()) < MAX_Z_ERROR_METERS
                && observation.pose.getX() <= FieldConstants.X_LENGTH_METERS
                && observation.pose.getX() >= 0
                && observation.pose.getY() <= FieldConstants.Y_LENGTH_METERS
                && observation.pose.getY() >= 0;
    }

    public static AprilTagVision createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real AprilTagVision on simulated robot", false);
        return new AprilTagVision(
                new AprilTagVisionIOPhoton(cameraConfigs[0]),
                new AprilTagVisionIOPhoton(cameraConfigs[1]),
                new AprilTagVisionIOPhoton(cameraConfigs[2]),
                new AprilTagVisionIOPhoton(cameraConfigs[3]));
    }

    public static AprilTagVision createSim() {
        if (Constants.currentMode != Constants.Mode.SIM)
            DriverStation.reportWarning("Using simulated AprilTagVision on real robot", false);
        return new AprilTagVision(
                new AprilTagVisionIOPhotonSim(cameraConfigs[0]),
                new AprilTagVisionIOPhotonSim(cameraConfigs[1]),
                new AprilTagVisionIOPhotonSim(cameraConfigs[2]),
                new AprilTagVisionIOPhotonSim(cameraConfigs[3]));
    }

    public static AprilTagVision createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy AprilTagVision on real robot", false);
        return new AprilTagVision(
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {},
                new AprilTagVisionIO() {});
    }
}
