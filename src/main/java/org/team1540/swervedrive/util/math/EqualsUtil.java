package org.team1540.swervedrive.util.math;

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    public static boolean epsilonEquals(Twist2d a, Twist2d b) {
        return epsilonEquals(a.dx, b.dx)
                && epsilonEquals(a.dy, b.dy)
                && epsilonEquals(a.dtheta, b.dtheta);
    }
}
