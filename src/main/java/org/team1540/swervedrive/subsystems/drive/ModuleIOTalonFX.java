package org.team1540.swervedrive.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.team1540.swervedrive.util.swerve.ModuleConfig;

import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final ModuleConfig moduleConfig;

    private final TalonFX drive;
    private final TalonFX turn;
    private final CANcoder cancoder;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;
    private final StatusSignal<Double> driveTemp;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnCurrent;
    private final StatusSignal<Double> turnTemp;

    private final VelocityVoltage driveVelocityReq = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut driveVoltageReq = new VoltageOut(0).withEnableFOC(true);

    private final PositionVoltage turnPositionReq = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut turnVoltageReq = new VoltageOut(0).withEnableFOC(true);

    public ModuleIOTalonFX(ModuleConfig moduleConfig, int index) {
        this.moduleConfig = moduleConfig;
        drive = moduleConfig.createDriveMotor(index);
        turn = moduleConfig.createTurnMotor(index);
        cancoder = moduleConfig.createCANcoder(index);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = drive.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drive, drive.getPosition());
        driveVelocity = drive.getVelocity();
        driveAppliedVolts = drive.getMotorVoltage();
        driveCurrent = drive.getSupplyCurrent();
        driveTemp = drive.getDeviceTemp();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turn.getPosition();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turn, turn.getPosition());
        turnVelocity = turn.getVelocity();
        turnAppliedVolts = turn.getMotorVoltage();
        turnCurrent = turn.getSupplyCurrent();
        turnTemp = turn.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Drivetrain.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                driveTemp,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent,
                turnTemp);
        drive.optimizeBusUtilization();
        turn.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveMotorConnected =
                BaseStatusSignal.refreshAll(
                        drivePosition,
                        driveVelocity,
                        driveAppliedVolts,
                        driveCurrent,
                        driveTemp).isOK();
        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent,
                turnTemp).isOK();
        inputs.turnEncoderConnected = turnAbsolutePosition.refresh().getStatus().isOK();

        inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
        inputs.driveTempCelsius = driveTemp.getValueAsDouble();

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = turnVelocity.getValueAsDouble();
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
        inputs.turnTempCelsius = turnTemp.getValueAsDouble();

        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRads =
                drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        drive.setControl(driveVelocityReq.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void setDriveVoltage(double volts) {
        drive.setControl(driveVoltageReq.withOutput(volts));
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        turn.setControl(turnPositionReq.withPosition(position.getRotations()));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turn.setControl(turnVoltageReq.withOutput(volts));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        drive.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        MotorOutputConfigs config = new MotorOutputConfigs();
        turn.getConfigurator().refresh(config);
        config.Inverted =
                moduleConfig.turnMotorInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turn.getConfigurator().apply(config);
    }

    @Override
    public void setDriveFF(double kS, double kV) {
        Slot0Configs config = new Slot0Configs();
        drive.getConfigurator().refresh(config);
        config.kS = kS;
        config.kV = kV;
        drive.getConfigurator().apply(config);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        Slot0Configs config = new Slot0Configs();
        drive.getConfigurator().refresh(config);
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        drive.getConfigurator().apply(config);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        Slot0Configs config = new Slot0Configs();
        turn.getConfigurator().refresh(config);
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        turn.getConfigurator().apply(config);
    }
}
