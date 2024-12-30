package org.team1540.swervedrive.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.swervedrive.Constants;

public class TurretIOSim implements TurretIO {
    private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim sim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, 0.25, Turret.GEAR_RATIO), MOTOR);
    private final ProfiledPIDController pid = Turret.GAINS.createProfiledPIDController(
            new TrapezoidProfile.Constraints(Turret.MAX_VELOCITY_RPS, Turret.MAX_ACCELERATION_RPS2));
    private SimpleMotorFeedforward ff = Turret.GAINS.createMotorFF();

    private double appliedVoltage = 0.0;
    private boolean isClosedLoop;

    public TurretIOSim() {
        pid.reset(sim.getAngularPositionRotations());
        sim.setState(0.0, 0.0);
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(sim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        Rotation2d position = Rotation2d.fromRotations(sim.getAngularPositionRotations());
        if (isClosedLoop) {
            appliedVoltage = pid.calculate(position.getRotations())
                    + ff.calculate(RotationsPerSecond.of(pid.getSetpoint().velocity))
                            .in(Volts);
        }
        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        appliedVoltage = MathUtil.clamp(appliedVoltage, -batteryVoltage, batteryVoltage);

        sim.setInputVoltage(appliedVoltage);
        sim.update(Constants.LOOP_PERIOD_SECS);

        if (sim.getAngularPositionRotations() > Turret.MAX_ANGLE.getRotations()) {
            sim.setState(Turret.MAX_ANGLE.getRadians(), 0.0);
        } else if (sim.getAngularPositionRotations() < Turret.MIN_ANGLE.getRotations()) {
            sim.setState(Turret.MIN_ANGLE.getRadians(), 0.0);
            sim.setAngularVelocity(0.0);
        }

        inputs.connected = true;
        inputs.timestamp = Timer.getFPGATimestamp();

        inputs.position = Rotation2d.fromRotations(sim.getAngularPositionRotations());
        inputs.velocityRPS = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
        inputs.appliedVolts = appliedVoltage;
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
        inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        isClosedLoop = false;
        appliedVoltage = voltage;
    }

    @Override
    public void setPosition(Rotation2d position) {
        isClosedLoop = true;
        pid.setGoal(position.getRotations());
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }

    @Override
    public void setFF(double kS, double kV) {
        ff = new SimpleMotorFeedforward(kS, kV);
    }
}
