package org.team1540.swervedrive.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) return;
        }
        DriverStation.reportWarning(
                "Failed to run a command on a CTRE device after " + maxAttempts + " attempts.", true);
    }

    public static double[] getSimulationOdometryTimestamps() {
        double[] odometryTimestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimestamps.length; i++) {
            odometryTimestamps[i] = Timer.getFPGATimestamp()
                    - 0.02
                    + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return odometryTimestamps;
    }
}
