package org.team1540.swervedrive.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        public boolean leaderConnected = false;
        public boolean followerConnected = false;

        public Rotation2d position = Rotation2d.kZero;
        public double velocityRadsPerSec = 0.0;
        public double[] appliedVolts = new double[] {0.0, 0.0};
        public double[] supplyCurrentAmps = new double[] {0.0, 0.0};
        public double[] statorCurrentAmps = new double[] {0.0, 0.0};
        public double[] tempCelsius = new double[] {0.0, 0.0};
    }

    default void updateInputs(ArmIOInputs inputs) {}

    default void setVoltage(double voltage) {}

    default void setPosition(Rotation2d position) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double p, double i, double d) {}

    default void setFF(double v, double s, double g) {}
}
