package org.team1540.swervedrive.subsystems.drive;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Queue;
import org.team1540.swervedrive.util.ClosedLoopConfig;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two DC motor sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private static final double simUpdatePeriod = 0.001;

    private final Notifier simNotifier;

    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private final PIDController turnPID;

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    private boolean isDriveClosedLoop;
    private boolean isTurnClosedLoop;

    public ModuleIOSim(SwerveModuleConstants constants) {
        // Create drive and turn sim models
        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
                DRIVE_GEARBOX);
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
                TURN_GEARBOX);
        var driveGains = new ClosedLoopConfig(constants.DriveMotorGains);
        var turnGains = new ClosedLoopConfig(constants.SteerMotorGains);
        drivePID = driveGains.createPIDController();
        driveFF = driveGains.createMotorFF();
        turnPID = turnGains.createPIDController();
        turnPID.enableContinuousInput(-0.5, 0.5);

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(driveSim::getAngularPositionRad);
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnSim::getAngularPositionRad);

        simNotifier = new Notifier(this::updateSimState);
        simNotifier.startPeriodic(simUpdatePeriod);
    }

    private void updateSimState() {
        if (isDriveClosedLoop)
            driveAppliedVolts = MathUtil.clamp(
                    drivePID.calculate(driveSim.getAngularVelocityRPM() / 60)
                            + driveFF.calculate(RotationsPerSecond.of(drivePID.getSetpoint()))
                                    .in(Volts),
                    -12.0,
                    12.0);
        if (isTurnClosedLoop)
            turnAppliedVolts = MathUtil.clamp(turnPID.calculate(turnSim.getAngularPositionRotations()), -12.0, 12.0);

        driveSim.setInputVoltage(driveAppliedVolts);
        turnSim.setInputVoltage(turnAppliedVolts);

        driveSim.update(simUpdatePeriod);
        turnSim.update(simUpdatePeriod);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveConnected = true;
        inputs.drivePositionRads = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadsPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadsPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble(value -> value).toArray();
        inputs.odometryDrivePositionsRads =
                drivePositionQueue.stream().mapToDouble(value -> value).toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRadians).toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        isDriveClosedLoop = true;
        drivePID.setSetpoint(Units.radiansToRotations(velocityRadPerSec));
    }

    @Override
    public void setDriveOpenLoop(double input) {
        isDriveClosedLoop = false;
        driveAppliedVolts = MathUtil.clamp(input, -12.0, 12.0);
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        isTurnClosedLoop = true;
        turnPID.setSetpoint(position.getRotations());
    }

    @Override
    public void setTurnOpenLoop(double input) {
        isTurnClosedLoop = false;
        turnAppliedVolts = MathUtil.clamp(input, -12.0, 12.0);
    }
}
