package org.team1540.swervedrive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;

    private static final boolean tuningMode = true;

    public static boolean isTuningMode() {
        return !DriverStation.isFMSAttached() && tuningMode;
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static final double ROBOT_MASS_KG = Units.lbsToKilograms(135);
    public static final double ROBOT_MOI_KG_M2 = 5.8;

    public static final double BUMPER_LENGTH_X_METERS = Units.inchesToMeters(37.0);
    public static final double BUMPER_LENGTH_Y_METERS = Units.inchesToMeters(33.0);
}
