package org.team1540.swervedrive.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public boolean teacupConnected = false;
        public boolean feederLeadConnected = false;
        public boolean feederFollowConnected = false;

        public boolean hasNote = false;

        public double teacupVelocityRPM = 0.0;
        public double teacupVoltage = 0.0;
        public double teacupSupplyCurrent = 0.0;
        public double teacupStatorCurrent = 0.0;
        public double teacupTemperatureCelsius = 0.0;

        public double feederVelocityRPM = 0.0;
        public double[] feederVoltage = {};
        public double[] feederSupplyCurrent = {};
        public double[] feederStatorCurrent = {};
        public double[] feederTemperatureCelsius = {};
    }

    default void updateInputs(FeederIOInputs inputs) {}

    default void setVoltage(double topVoltage, double bottomVoltage) {}
}
