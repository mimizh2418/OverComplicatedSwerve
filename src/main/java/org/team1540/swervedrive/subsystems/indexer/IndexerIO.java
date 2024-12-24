package org.team1540.swervedrive.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public boolean intakeConnected = false;
        public boolean feederConnected = false;
        public boolean hasNote = false;

        public double intakeAppliedVolts = 0.0;
        public double intakeStatorCurrentAmps = 0.0;
        public double intakeSupplyCurrentAmps = 0.0;
        public double intakeTempCelsius = 0.0;
        public double intakeVelocityRPM = 0.0;

        public double feederAppliedVolts = 0.0;
        public double feederStatorCurrentAmps = 0.0;
        public double feederSupplyCurrentAmps = 0.0;
        public double feederTempCelsius = 0.0;
        public double feederVelocityRPM = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setIntakeVoltage(double voltage) {}

    default void setFeederVoltage(double voltage) {}
}
