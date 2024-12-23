package org.team1540.swervedrive.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class JoystickUtil {
    public static double smartDeadzone(double rawInput, double deadzone) {
        double scalar = 1 / (1 - deadzone);
        if (rawInput > deadzone) {
            return (rawInput - deadzone) * scalar;
        }
        if (rawInput < -deadzone) {
            return (rawInput + deadzone) * scalar;
        }
        return 0;
    }

    public static Translation2d getJoystickTranslation(double rawX, double rawY, double deadzone) {
        double linearMagnitude = JoystickUtil.smartDeadzone(Math.hypot(rawX, rawY), deadzone);
        Rotation2d linearDirection = new Rotation2d(rawX, rawY);
        return new Pose2d(Translation2d.kZero, linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }
}
