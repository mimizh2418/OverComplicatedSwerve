package org.team1540.swervedrive.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public boolean leftConnected = false;
        public boolean rightConnected = false;

        public double leftAppliedVolts = 0.0;
        public double leftStatorCurrentAmps = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;
        public double leftVelocityRPM = 0.0;

        public double rightAppliedVolts = 0.0;
        public double rightStatorCurrentAmps = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;
        public double rightVelocityRPM = 0.0;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void setVoltage(double leftVolts, double rightVolts) {}

    default void setVelocity(double leftRPM, double rightRPM) {}

    default void setPID(double p, double i, double d) {}

    default void setFF(double s, double v) {}
}
