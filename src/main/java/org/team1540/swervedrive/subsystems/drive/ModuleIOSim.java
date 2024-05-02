package org.team1540.swervedrive.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.util.swerve.ModuleConfig;

import java.util.Arrays;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private final PIDController drivePID;
    private SimpleMotorFeedforward driveFF;
    private final PIDController turnPID;

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private boolean isDriveClosedLoop;
    private boolean isTurnClosedLoop;

    public ModuleIOSim(ModuleConfig config) {
        driveSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), config.driveGearRatio(), 0.025);
        turnSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), config.turnGearRatio(), 0.004);
        drivePID = config.driveVelocityGains().createPIDController();
        driveFF = config.driveVelocityGains().createMotorFF();
        turnPID = config.turnPositionGains().createPIDController();
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (isDriveClosedLoop)
            driveAppliedVolts =
                    MathUtil.clamp(drivePID.calculate(Units.radiansToRotations(inputs.driveVelocityRadPerSec))
                            + driveFF.calculate(drivePID.getSetpoint()), -12.0, 12.0);
        if (isTurnClosedLoop)
            turnAppliedVolts = MathUtil.clamp(turnPID.calculate(inputs.turnPosition.getRadians()), -12.0, 12.0);

        driveSim.setInputVoltage(driveAppliedVolts);
        turnSim.setInputVoltage(turnAppliedVolts);

        driveSim.update(Constants.LOOP_PERIOD_SECS);
        turnSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.drivePositionRads = driveSim.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.turnAbsolutePosition =
                new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
        inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

        inputs.odometryTimestamps = new double[]{Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRads = new double[]{inputs.drivePositionRads};
        inputs.odometryTurnPositions = new Rotation2d[]{inputs.turnPosition};
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        isDriveClosedLoop = true;
        drivePID.setSetpoint(Units.radiansToRotations(velocityRadPerSec));
    }

    @Override
    public void setDriveVoltage(double volts) {
        isDriveClosedLoop = false;
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        isTurnClosedLoop = true;
        turnPID.setSetpoint(position.getRadians());
    }

    @Override
    public void setTurnVoltage(double volts) {
        isTurnClosedLoop = false;
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setDriveFF(double kS, double kV) {
        driveFF = new SimpleMotorFeedforward(kS, kV);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        drivePID.setPID(kP, kI, kD);
    }

    public void setTurnPID(double kP, double kI, double kD) {
        turnPID.setPID(kP, kI, kD);
    }
}
