package org.team1540.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FieldConstants {
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public static final int NUM_APRILTAGS = TAG_LAYOUT.getTags().size();

    public static final double X_LENGTH_METERS = TAG_LAYOUT.getFieldLength();
    public static final double Y_LENGTH_METERS = TAG_LAYOUT.getFieldWidth();

    public static boolean inField(Pose2d pose) {
        return pose.getX() <= FieldConstants.X_LENGTH_METERS
                && pose.getX() >= 0
                && pose.getY() <= FieldConstants.Y_LENGTH_METERS
                && pose.getY() >= 0;
    }

    public static final Pose2d MIDFIELD = new Pose2d(X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2, Rotation2d.kZero);
}
