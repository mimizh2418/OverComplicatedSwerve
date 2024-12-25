package org.team1540.swervedrive.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.*;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;

/**
 * The robot knows where it is at all times. It knows this because it knows where it would have been when it saw what it
 * saw. By combining where what it saw should be, with where it must have been to see what it saw, relative to wherever
 * what it saw is, it knows where it was when it saw what it saw.
 *
 * <p> To know where it is, instead of where it was, it must know how it got from where it was to where it is, wherever
 * where it was, was. It knows this because it knows where its wheels went when it went from where it was to where it
 * is.
 *
 * <p> By starting at where it was when it saw what it saw, and going the way it went, it knows where it should be now.
 * In the event that where it is when it next sees what it sees is not where it would be if it went the way it went from
 * where it was, the robot has acquired an error. With a weighted average of where it would have been with where it was
 * when it saw what it saw, it can correct where it was, and therefore where it should be — which is called pose.
 */
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

    public record CameraConfig(
            String name,
            Transform3d robotToCamera,
            Rotation2d estimatedDiagonalFOV,
            Dimension resolution,
            double trustCoeff) {}

    private static final CameraConfig[] cameraConfigs = new CameraConfig[] {
        new CameraConfig(
                "northstar-0",
                new Transform3d(
                        Units.inchesToMeters(8.875),
                        Units.inchesToMeters(10.5),
                        Units.inchesToMeters(8.25),
                        new Rotation3d(0.0, Math.toRadians(-28.125), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Math.toRadians(30)))),
                Rotation2d.fromDegrees(75),
                new Dimension(1600, 1200),
                1.0),
        new CameraConfig(
                "northstar-1",
                new Transform3d(
                        Units.inchesToMeters(3.25),
                        Units.inchesToMeters(5.0),
                        Units.inchesToMeters(6.4),
                        new Rotation3d(0.0, Math.toRadians(-16.876), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Math.toRadians(-4.709)))),
                Rotation2d.fromDegrees(45),
                new Dimension(1600, 1200),
                1.5),
        new CameraConfig(
                "northstar-2",
                new Transform3d(
                        Units.inchesToMeters(8.875),
                        Units.inchesToMeters(-10.5),
                        Units.inchesToMeters(8.25),
                        new Rotation3d(0.0, Math.toRadians(-28.125), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Math.toRadians(-30.0)))),
                Rotation2d.fromDegrees(75),
                new Dimension(1600, 1200),
                1.0),
        new CameraConfig(
                "northstar-3",
                new Transform3d(
                        Units.inchesToMeters(-16.0),
                        Units.inchesToMeters(-12.0),
                        Units.inchesToMeters(8.5),
                        new Rotation3d(0.0, Units.degreesToRadians(-33.75), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(176.386)))),
                Rotation2d.fromDegrees(90),
                new Dimension(1600, 1200),
                0.8)
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
                double stdDevFactor = (observation.avgTagDistanceMeters * observation.avgTagDistanceMeters)
                        / observation.numTags
                        * (1 / cameraConfigs[i].trustCoeff);
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
                && FieldConstants.inField(observation.pose.toPose2d());
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
