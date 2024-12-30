package org.team1540.swervedrive;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import org.team1540.swervedrive.subsystems.intake.Intake;
import org.team1540.swervedrive.util.AllianceFlipUtil;

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

    private static final double BLUE_WING_X_METERS = Units.inchesToMeters(229.201);
    private static final double RED_WING_X_METERS = X_LENGTH_METERS - BLUE_WING_X_METERS;

    public static boolean inOwnWing(Pose2d pose) {
        if (!inField(pose)) return false;
        return AllianceFlipUtil.maybeFlipPose(pose).getX() <= BLUE_WING_X_METERS;
    }

    public static boolean inOpponentWing(Pose2d pose) {
        if (!inField(pose)) return false;
        return AllianceFlipUtil.maybeFlipPose(pose).getX() >= RED_WING_X_METERS;
    }

    private static final Pose2d BLUE_SPEAKER =
            new Pose2d(Units.inchesToMeters(8.861), Units.inchesToMeters(218), Rotation2d.kZero);

    public static Pose2d getSpeakerPose() {
        return AllianceFlipUtil.maybeFlipPose(BLUE_SPEAKER);
    }

    private static final Pose2d SUBWOOFER_POSE =
            new Pose2d(Units.inchesToMeters(36.179), Units.inchesToMeters(218), Rotation2d.kZero);

    public static Pose2d getSubwooferStartingPose() {
        return AllianceFlipUtil.maybeFlipPose(SUBWOOFER_POSE)
                .transformBy(new Transform2d(
                        (Constants.BUMPER_LENGTH_X_METERS - Intake.EXTENSION_METERS) / 2, 0, Rotation2d.kZero));
    }

    private static final Pose2d BLUE_PASS_TARGET = BLUE_SPEAKER.plus(new Transform2d(0.5, 1.0, Rotation2d.kZero));
    private static final Pose2d RED_PASS_TARGET = FlippingUtil.flipFieldPose(BLUE_PASS_TARGET);

    public static Pose2d getPassingTarget() {
        return AllianceFlipUtil.shouldFlip() ? RED_PASS_TARGET : BLUE_PASS_TARGET;
    }
}
