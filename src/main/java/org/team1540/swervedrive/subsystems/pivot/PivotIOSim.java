package org.team1540.swervedrive.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.swervedrive.Constants;

public class PivotIOSim implements PivotIO {
    private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            MOTOR,
            Pivot.GEAR_RATIO,
            0.5,
            Pivot.LENGTH_METERS,
            Pivot.MIN_ANGLE.getRadians(),
            Pivot.MAX_ANGLE.getRadians(),
            false,
            0.0);

    private final ProfiledPIDController pid = Pivot.GAINS.createProfiledPIDController(
            new TrapezoidProfile.Constraints(Pivot.MAX_VELOCITY_RPS, Pivot.MAX_ACCELERATION_RPS2));
    private SimpleMotorFeedforward ff = Pivot.GAINS.createMotorFF();

    private boolean isClosedLoop;
    private double voltage = 0.0;

    public PivotIOSim() {
        pid.reset(sim.getAngleRads());
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(sim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        Rotation2d position = Rotation2d.fromRadians(sim.getAngleRads());
        if (isClosedLoop) {
            voltage = pid.calculate(position.getRotations())
                    + ff.calculate(RotationsPerSecond.of(pid.getSetpoint().velocity))
                            .in(Volts);
        }

        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        voltage = MathUtil.clamp(voltage, -batteryVoltage, batteryVoltage);

        sim.setInputVoltage(voltage);
        sim.update(Constants.LOOP_PERIOD_SECS);

        inputs.connected = true;

        inputs.position = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.velocityRPS = Units.radiansToRotations(sim.getVelocityRadPerSec());
        inputs.appliedVolts = voltage;
        inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
        inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        isClosedLoop = false;
        this.voltage = voltage;
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
