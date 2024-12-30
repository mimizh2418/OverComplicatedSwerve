package org.team1540.swervedrive.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public boolean topConnected = false;
        public boolean bottomConnected = false;

        public boolean hasNote = false;

        public double topVelocityRPM = 0.0;
        public double topVoltage = 0.0;
        public double topSupplyCurrent = 0.0;
        public double topStatorCurrent = 0.0;
        public double topTemperatureCelsius = 0.0;

        public double bottomVelocityRPM = 0.0;
        public double bottomVoltage = 0.0;
        public double bottomSupplyCurrent = 0.0;
        public double bottomStatorCurrent = 0.0;
        public double bottomTemperatureCelsius = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setVoltage(double topVoltage, double bottomVoltage) {}
}
