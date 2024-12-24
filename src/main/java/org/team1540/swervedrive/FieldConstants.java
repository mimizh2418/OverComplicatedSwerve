package org.team1540.swervedrive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.team1540.swervedrive.util.AllianceFlipUtil;

public final class FieldConstants {
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public static final int NUM_APRILTAGS = TAG_LAYOUT.getTags().size();

    public static final double X_LENGTH_METERS = TAG_LAYOUT.getFieldLength();
    public static final double Y_LENGTH_METERS = TAG_LAYOUT.getFieldWidth();

    public static final Pose2d MIDFIELD = new Pose2d(X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2, Rotation2d.kZero);

    private static final Pose2d BLUE_SPEAKER =
            new Pose2d(Units.inchesToMeters(8.861), Units.inchesToMeters(218), Rotation2d.kZero);
    private static final Pose2d RED_SPEAkER = FlippingUtil.flipFieldPose(BLUE_SPEAKER);

    public static Pose2d getSpeakerPose() {
        return AllianceFlipUtil.shouldFlip() ? RED_SPEAkER : BLUE_SPEAKER;
    }
}
