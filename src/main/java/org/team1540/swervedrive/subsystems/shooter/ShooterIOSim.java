package org.team1540.swervedrive.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.SimState;

public class ShooterIOSim implements ShooterIO {
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);

    private final FlywheelSim leftSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(GEARBOX, 0.000205433, Shooter.GEARING), GEARBOX);
    private final FlywheelSim rightSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(GEARBOX, 0.000205433, Shooter.GEARING), GEARBOX);

    private final PIDController leftPID = Shooter.GAINS.createPIDController();
    private final PIDController rightPID = Shooter.GAINS.createPIDController();
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Shooter.GAINS.kS, Shooter.GAINS.kV);

    private double leftVoltage = 0.0;
    private double rightVoltage = 0.0;

    private boolean isClosedLoop;

    public ShooterIOSim() {
        SimulatedBattery.addElectricalAppliances(
                () -> Amps.of(leftSim.getCurrentDrawAmps() + rightSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        double leftVelocityRPS = leftSim.getAngularVelocityRPM() / 60.0;
        double rightVelocityRPS = rightSim.getAngularVelocityRPM() / 60.0;
        if (isClosedLoop) {
            leftVoltage = leftPID.calculate(leftVelocityRPS)
                    + ff.calculate(RotationsPerSecond.of(leftPID.getSetpoint())).in(Volts);
            rightVoltage = rightPID.calculate(rightVelocityRPS)
                    + ff.calculate(RotationsPerSecond.of(rightPID.getSetpoint()))
                            .in(Volts);
        }

        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        leftVoltage = MathUtil.clamp(leftVoltage, -batteryVoltage, batteryVoltage);
        rightVoltage = MathUtil.clamp(rightVoltage, -batteryVoltage, batteryVoltage);

        leftSim.setInputVoltage(leftVoltage);
        rightSim.setInputVoltage(rightVoltage);
        leftSim.update(Constants.LOOP_PERIOD_SECS);
        rightSim.update(Constants.LOOP_PERIOD_SECS);

        SimState.getInstance()
                .addShooterVelocityData(leftSim.getAngularVelocityRPM(), rightSim.getAngularVelocityRPM());

        inputs.leftConnected = true;
        inputs.rightConnected = true;

        inputs.leftAppliedVolts = leftVoltage;
        inputs.leftStatorCurrentAmps = leftSim.getCurrentDrawAmps();
        inputs.leftSupplyCurrentAmps = leftSim.getCurrentDrawAmps();
        inputs.leftVelocityRPM = leftSim.getAngularVelocityRPM();

        inputs.rightAppliedVolts = rightVoltage;
        inputs.rightStatorCurrentAmps = rightSim.getCurrentDrawAmps();
        inputs.rightSupplyCurrentAmps = rightSim.getCurrentDrawAmps();
        inputs.rightVelocityRPM = rightSim.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        isClosedLoop = false;
        leftVoltage = leftVolts;
        rightVoltage = rightVolts;
    }

    @Override
    public void setVelocity(double leftRPM, double rightRPM) {
        isClosedLoop = true;
        leftPID.setSetpoint(leftRPM / 60.0);
        rightPID.setSetpoint(rightRPM / 60.0);
    }

    @Override
    public void setPID(double p, double i, double d) {
        leftPID.setPID(p, i, d);
        rightPID.setPID(p, i, d);
    }

    @Override
    public void setFF(double s, double v) {
        ff = new SimpleMotorFeedforward(s, v);
    }
}