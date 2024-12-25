package org.team1540.swervedrive.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.swervedrive.Constants;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2),
            Arm.GEAR_RATIO,
            1.06328,
            Arm.LENGTH_METERS,
            Arm.MIN_ANGLE.getRadians(),
            Arm.MAX_ANGLE.getRadians(),
            false,
            0.0);

    private final ProfiledPIDController pid = Arm.GAINS.createProfiledPIDController(
            new TrapezoidProfile.Constraints(Arm.MAX_VELOCITY_RPS, Arm.MAX_ACCELERATION_RPS2));
    private ArmFeedforward ff = Arm.GAINS.createArmFF();

    private double appliedVoltage = 0.0;
    private boolean isClosedLoop;

    public ArmIOSim() {
        sim.setState(Arm.MIN_ANGLE.getRadians(), 0.0);
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(sim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        Rotation2d position = Rotation2d.fromRadians(sim.getAngleRads());
        if (isClosedLoop) {
            appliedVoltage = pid.calculate(position.getRotations())
                    + ff.calculate(position.getMeasure(), RotationsPerSecond.of(pid.getSetpoint().velocity))
                            .in(Volts);
        }

        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        appliedVoltage = MathUtil.clamp(appliedVoltage, -batteryVoltage, batteryVoltage);

        sim.setInputVoltage(appliedVoltage);
        sim.update(Constants.LOOP_PERIOD_SECS);

        inputs.leaderConnected = true;
        inputs.followerConnected = true;
        inputs.encoderConnected = true;

        inputs.position = Rotation2d.fromRadians(sim.getAngleRads());
        inputs.absolutePosition = inputs.position;
        inputs.velocityRPS = Units.radiansToRotations(sim.getVelocityRadPerSec());
        inputs.appliedVolts = new double[] {appliedVoltage, appliedVoltage};
        inputs.supplyCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.statorCurrentAmps = new double[] {sim.getCurrentDrawAmps()};

        if (isClosedLoop) {
            appliedVoltage = pid.calculate(inputs.position.getRotations())
                    + ff.calculate(inputs.position.getMeasure(), RotationsPerSecond.of(inputs.velocityRPS))
                            .in(Volts);
        }
        sim.setInputVoltage(appliedVoltage);
        sim.update(Constants.LOOP_PERIOD_SECS);
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        isClosedLoop = false;
    }

    @Override
    public void setPosition(Rotation2d position) {
        pid.reset(Units.radiansToRotations(sim.getAngleRads()), Units.radiansToRotations(sim.getVelocityRadPerSec()));
        pid.setGoal(position.getRotations());
        isClosedLoop = true;
    }

    @Override
    public void setPID(double p, double i, double d) {
        pid.setPID(p, i, d);
    }

    @Override
    public void setFF(double s, double v, double g) {
        ff = new ArmFeedforward(s, g, v);
    }
}
