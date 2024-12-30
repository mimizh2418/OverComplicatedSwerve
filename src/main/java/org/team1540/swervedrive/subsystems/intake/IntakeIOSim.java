package org.team1540.swervedrive.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.SimState;

public class IntakeIOSim implements IntakeIO {
    private static final DCMotor TOP_MOTOR = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor BOTTOM_MOTOR = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim topSim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(TOP_MOTOR, 0.005, Intake.GEAR_RATIO), TOP_MOTOR);
    private final DCMotorSim bottomSim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(BOTTOM_MOTOR, 0.001, Intake.GEAR_RATIO), BOTTOM_MOTOR);

    private double topVoltage = 0.0;
    private double bottomVoltage = 0.0;

    public IntakeIOSim() {
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(topSim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(bottomSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        topVoltage = MathUtil.clamp(topVoltage, -batteryVoltage, batteryVoltage);
        bottomVoltage = MathUtil.clamp(bottomVoltage, -batteryVoltage, batteryVoltage);

        topSim.setInputVoltage(topVoltage);
        bottomSim.setInputVoltage(bottomVoltage);
        topSim.update(Constants.LOOP_PERIOD_SECS);
        bottomSim.update(Constants.LOOP_PERIOD_SECS);

        SimState.getInstance().addIntakeData(topVoltage, bottomVoltage);

        inputs.topConnected = true;
        inputs.bottomConnected = true;

        inputs.hasNote = SimState.getInstance().noteInIntake();

        inputs.topVelocityRPM = topSim.getAngularVelocityRPM();
        inputs.topVoltage = topVoltage;
        inputs.topSupplyCurrent = topSim.getCurrentDrawAmps();
        inputs.topStatorCurrent = topSim.getCurrentDrawAmps();

        inputs.bottomVelocityRPM = bottomSim.getAngularVelocityRPM();
        inputs.bottomVoltage = bottomVoltage;
        inputs.bottomSupplyCurrent = bottomSim.getCurrentDrawAmps();
        inputs.bottomStatorCurrent = bottomSim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double topVoltage, double bottomVoltage) {
        this.topVoltage = topVoltage;
        this.bottomVoltage = bottomVoltage;
    }
}
