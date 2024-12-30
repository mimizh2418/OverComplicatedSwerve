package org.team1540.swervedrive.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.SimState;

public class FeederIOSim implements FeederIO {
    private static final DCMotor TEACUP_MOTOR = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor FEEDER_MOTOR = DCMotor.getKrakenX60Foc(2);

    private final DCMotorSim teacupSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TEACUP_MOTOR, 0.005, Feeder.TEACUP_GEAR_RATIO), TEACUP_MOTOR);
    private final DCMotorSim feederSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(FEEDER_MOTOR, 0.005, Feeder.FEEDER_GEAR_RATIO), FEEDER_MOTOR);

    private double teacupVoltage = 0.0;
    private double feederVoltage = 0.0;

    public FeederIOSim() {
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(teacupSim.getCurrentDrawAmps()));
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(feederSim.getCurrentDrawAmps()));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        double batteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
        teacupVoltage = MathUtil.clamp(teacupVoltage, -batteryVoltage, batteryVoltage);
        feederVoltage = MathUtil.clamp(feederVoltage, -batteryVoltage, batteryVoltage);

        teacupSim.setInputVoltage(teacupVoltage);
        feederSim.setInputVoltage(feederVoltage);
        teacupSim.update(Constants.LOOP_PERIOD_SECS);
        feederSim.update(Constants.LOOP_PERIOD_SECS);

        SimState.getInstance().addFeederData(teacupVoltage, feederVoltage);

        inputs.teacupConnected = true;
        inputs.feederLeadConnected = true;
        inputs.feederFollowConnected = true;

        inputs.hasNote = SimState.getInstance().noteInFeeder();

        inputs.teacupVelocityRPM = teacupSim.getAngularVelocityRPM();
        inputs.teacupVoltage = teacupVoltage;
        inputs.teacupSupplyCurrent = teacupSim.getCurrentDrawAmps();
        inputs.teacupStatorCurrent = teacupSim.getCurrentDrawAmps();

        inputs.feederVelocityRPM = feederSim.getAngularVelocityRPM();
        inputs.feederVoltage = new double[] {feederVoltage, feederVoltage};
        inputs.feederSupplyCurrent = new double[] {feederSim.getCurrentDrawAmps()};
        inputs.feederStatorCurrent = new double[] {feederSim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double teacupVoltage, double feederVoltage) {
        this.teacupVoltage = teacupVoltage;
        this.feederVoltage = feederVoltage;
    }
}
