package org.team1540.swervedrive.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;

public class AprilTagVisionIOPhotonSim extends AprilTagVisionIOPhoton {
    private static VisionSystemSim visionSim;

    private final PhotonCameraSim cameraSim;

    public AprilTagVisionIOPhotonSim(AprilTagVision.CameraConfig config) {
        super(config);

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.TAG_LAYOUT);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(
                config.resolution().width, config.resolution().height, config.estimatedDiagonalFOV());
        cameraProperties.setCalibError(0.25, 0.125);
        cameraProperties.setFPS(30.0);
        cameraProperties.setAvgLatencyMs(15.0);
        cameraProperties.setLatencyStdDevMs(7.5);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSim.update(RobotState.getInstance().getSimulatedRobotPose());
        super.updateInputs(inputs);
    }
}
