package org.team1540.swervedrive.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.team1540.swervedrive.util.ClosedLoopConfig;

public record ModuleConfig(
        double driveGearRatio,
        double turnGearRatio,
        boolean turnMotorInverted,
        double slipCurrent,
        ClosedLoopConfig driveVelocityGains,
        ClosedLoopConfig turnPositionGains,
        String canbus) {
    public TalonFX createDriveMotor(int moduleId) {
        int motorId = moduleId + 30;
        TalonFX driveMotor = new TalonFX(motorId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = slipCurrent;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = driveGearRatio;
        config.Slot0 = driveVelocityGains.createCTREConfigs();

        driveMotor.getConfigurator().apply(config);
        return driveMotor;
    }

    public TalonFX createTurnMotor(int moduleId) {
        int motorId = moduleId + 20;
        int cancoderId = moduleId + 10;
        TalonFX turnMotor = new TalonFX(motorId, canbus);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.MotorOutput.Inverted = turnMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.Feedback.FeedbackRemoteSensorID = cancoderId;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.RotorToSensorRatio = turnGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.Slot0 = turnPositionGains.createCTREConfigs();

        turnMotor.getConfigurator().apply(config);
        return turnMotor;
    }

    public CANcoder createCANcoder(int moduleId) {
        int cancoderId = moduleId + 10;
        CANcoder cancoder = new CANcoder(cancoderId, canbus);
        CANcoderConfiguration config = new CANcoderConfiguration();
        cancoder.getConfigurator().refresh(config);
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cancoder.getConfigurator().apply(config);
        return cancoder;
    }
}
