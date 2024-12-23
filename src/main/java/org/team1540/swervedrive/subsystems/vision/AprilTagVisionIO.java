package org.team1540.swervedrive.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
    @AutoLog
    class AprilTagVisionIOInputs {
        public boolean connected = false;
        public AprilTagVision.TagObservation[] latestTargetObservations = new AprilTagVision.TagObservation[0];
        public AprilTagVision.PoseObservation[] poseObservations = new AprilTagVision.PoseObservation[0];
    }

    default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
