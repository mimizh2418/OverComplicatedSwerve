package org.team1540.swervedrive.util;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipUtil {
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
    }

    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) return GeometryUtil.flipFieldPose(pose);
        return pose;
    }

    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) return GeometryUtil.flipFieldPosition(translation);
        return translation;
    }

    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) return GeometryUtil.flipFieldRotation(rotation);
        return rotation;
    }
}
