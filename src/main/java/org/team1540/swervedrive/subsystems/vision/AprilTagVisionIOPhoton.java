package org.team1540.swervedrive.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.*;
import java.util.stream.IntStream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team1540.swervedrive.FieldConstants;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {
    protected final Transform3d robotToCamera;
    protected final PhotonCamera camera;
    protected final PhotonPoseEstimator poseEstimator;

    public AprilTagVisionIOPhoton(AprilTagVision.CameraConfig cameraConfig) {
        this.robotToCamera = cameraConfig.robotToCamera();
        camera = new PhotonCamera(cameraConfig.name());
        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.TAG_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        Transform3d[] tagObservationsById = new Transform3d[FieldConstants.NUM_APRILTAGS + 1];
        List<AprilTagVision.PoseObservation> poseObservations = new ArrayList<>(results.size());
        for (PhotonPipelineResult result : results) {
            double averageTagDistanceMeters = 0;
            int numValidTags = 0;
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() <= FieldConstants.NUM_APRILTAGS) {
                    Transform3d cameraToTag = target.getBestCameraToTarget();
                    tagObservationsById[target.getFiducialId()] = robotToCamera.plus(cameraToTag);
                    averageTagDistanceMeters += cameraToTag.getTranslation().getNorm();
                    numValidTags++;
                }
            }
            if (numValidTags > 0) averageTagDistanceMeters /= numValidTags;

            Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update(result);
            if (poseEstimatorResult.isPresent()) {
                EstimatedRobotPose poseEstimate = poseEstimatorResult.get();

                poseObservations.add(new AprilTagVision.PoseObservation(
                        poseEstimate.timestampSeconds,
                        poseEstimate.estimatedPose,
                        poseEstimate.targetsUsed.size(),
                        averageTagDistanceMeters,
                        poseEstimate.targetsUsed.get(0)
                                .poseAmbiguity, // Ambiguity is only relevant for single-target estimates
                        AprilTagVision.PoseObservationSource.PHOTONVISION));
            }
        }

        inputs.latestTargetObservations = IntStream.range(0, tagObservationsById.length)
                .filter(i -> tagObservationsById[i] != null)
                .mapToObj(i -> new AprilTagVision.TagObservation(
                        i,
                        tagObservationsById[i],
                        FieldConstants.TAG_LAYOUT.getTagPose(i).orElse(Pose3d.kZero)))
                .toArray(AprilTagVision.TagObservation[]::new);
        inputs.poseObservations = poseObservations.toArray(new AprilTagVision.PoseObservation[0]);
    }
}
