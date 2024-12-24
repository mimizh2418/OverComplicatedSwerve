package org.team1540.swervedrive.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.team1540.swervedrive.util.PhoenixUtil;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX leader = new TalonFX(Arm.LEADER_ID);
    private final TalonFX follower = new TalonFX(Arm.FOLLOWER_ID);
    private final CANcoder encoder = new CANcoder(Arm.ENCODER_ID);

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final VoltageOut voltageReq = new VoltageOut(0);
    private final MotionMagicVoltage positionReq = new MotionMagicVoltage(0.0).withSlot(0);
    private final Follower followerReq = new Follower(leader.getDeviceID(), true);

    private final StatusSignal<Angle> position = leader.getPosition();
    private final StatusSignal<Angle> absolutePosition = encoder.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> velocity = leader.getVelocity();
    private final List<StatusSignal<Voltage>> appliedVoltage =
            List.of(leader.getMotorVoltage(), follower.getMotorVoltage());
    private final List<StatusSignal<Current>> supplyCurrent =
            List.of(leader.getSupplyCurrent(), follower.getSupplyCurrent());
    private final List<StatusSignal<Current>> statorCurrent =
            List.of(leader.getStatorCurrent(), follower.getStatorCurrent());
    private final List<StatusSignal<Temperature>> temperature =
            List.of(leader.getDeviceTemp(), follower.getDeviceTemp());

    private final Executor brakeModeExecutor = Executors.newFixedThreadPool(4);

    public ArmIOTalonFX() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = Arm.ENCODER_OFFSET_ROTS;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig));

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = Arm.ENCODER_ID;
        motorConfig.Feedback.SensorToMechanismRatio = 1.0;
        motorConfig.Feedback.RotorToSensorRatio = Arm.GEAR_RATIO;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Arm.MAX_ANGLE.getRotations();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Arm.MIN_ANGLE.getRotations();

        motorConfig.Slot0 = Arm.GAINS.createCTREConfigs();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = Arm.MAX_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = Arm.MAX_ACCELERATION_RPS2;

        PhoenixUtil.tryUntilOk(5, () -> leader.getConfigurator().apply(motorConfig));
        PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(motorConfig));

        PhoenixUtil.tryUntilOk(5, () -> follower.setControl(followerReq));

        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                position,
                absolutePosition,
                velocity,
                appliedVoltage.get(0),
                appliedVoltage.get(1),
                supplyCurrent.get(0),
                supplyCurrent.get(1),
                statorCurrent.get(0),
                statorCurrent.get(1),
                temperature.get(0),
                temperature.get(1));

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();
        encoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.leaderConnected = BaseStatusSignal.refreshAll(
                        position,
                        velocity,
                        appliedVoltage.get(0),
                        supplyCurrent.get(0),
                        statorCurrent.get(0),
                        temperature.get(0))
                .isOK();
        inputs.followerConnected = BaseStatusSignal.refreshAll(
                        appliedVoltage.get(1), supplyCurrent.get(1), statorCurrent.get(1), temperature.get(1))
                .isOK();
        inputs.encoderConnected = BaseStatusSignal.refreshAll(absolutePosition).isOK();

        inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.absolutePosition = Rotation2d.fromRotations(absolutePosition.getValueAsDouble());
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.supplyCurrentAmps = supplyCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.statorCurrentAmps = statorCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.tempCelsius =
                temperature.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    }

    @Override
    public void setVoltage(double voltage) {
        leader.setControl(voltageReq.withOutput(voltage));
    }

    @Override
    public void setPosition(Rotation2d position) {
        leader.setControl(positionReq.withPosition(position.getRotations()));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            motorConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            synchronized (leader) {
                leader.getConfigurator().apply(motorConfig);
            }
            synchronized (follower) {
                follower.getConfigurator().apply(motorConfig);
            }
        });
    }

    @Override
    public void setPID(double p, double i, double d) {
        Slot0Configs configs = new Slot0Configs();
        leader.getConfigurator().refresh(configs);
        configs.kP = p;
        configs.kI = i;
        configs.kD = d;
        leader.getConfigurator().apply(configs);
        follower.getConfigurator().apply(configs);
    }

    @Override
    public void setFF(double s, double v, double g) {
        Slot0Configs configs = new Slot0Configs();
        leader.getConfigurator().refresh(configs);
        configs.kS = s;
        configs.kV = v;
        configs.kG = g;
        leader.getConfigurator().apply(configs);
        follower.getConfigurator().apply(configs);
    }
}
