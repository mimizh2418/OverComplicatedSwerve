package org.team1540.swervedrive.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public boolean connected = false;

        public double timestamp = 0.0;
        public Rotation2d position = Rotation2d.kZero;
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double statorCurrentAmps = 0.0;
        public double temperatureCelsius = 0.0;
    }

    default void updateInputs(TurretIOInputs inputs) {}

    default void setVoltage(double voltage) {}

    default void setPosition(Rotation2d position) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kS, double kV) {}
}
