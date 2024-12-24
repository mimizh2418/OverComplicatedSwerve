package org.team1540.swervedrive.subsystems.indexer;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.RobotState;

public class IndexerIOSim implements IndexerIO {
    private static final DCMotor INTAKE_GEARBOX = DCMotor.getKrakenX60Foc(1).withReduction(Indexer.INTAKE_GEARING);
    private static final DCMotor FEEDER_GEARBOX = DCMotor.getKrakenX60Foc(1).withReduction(Indexer.FEEDER_GEARING);

    private static final double INTAKE_INTEGRAL_COEFF = 0.15 / Indexer.INTAKE_GEARING;
    private static final double FEED_INTEGRAL_COEFF = 0.15 / Indexer.FEEDER_GEARING;

    private final DCMotorSim intakeMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_GEARBOX, 0.001, Indexer.INTAKE_GEARING), INTAKE_GEARBOX);
    private final DCMotorSim feederMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(FEEDER_GEARBOX, 0.001, Indexer.FEEDER_GEARING), FEEDER_GEARBOX);
    private final IntakeSimulation intakeSim;

    private double intakeVoltage = 0.0;
    private double feederVoltage = 0.0;

    private double intakeVoltageIntegral = 0.0;
    private double feederVoltageIntegral = 0.0;

    public IndexerIOSim(SwerveDriveSimulation driveSim) {
        intakeSim = new IntakeSimulation(
                "Note", driveSim, Meters.of(Indexer.INTAKE_WIDTH_METERS), IntakeSimulation.IntakeSide.BACK, 1);
        intakeSim.register(SimulatedArena.getInstance());
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        intakeMotorSim.setInputVoltage(intakeVoltage);
        feederMotorSim.setInputVoltage(feederVoltage);
        intakeMotorSim.update(Constants.LOOP_PERIOD_SECS);
        feederMotorSim.update(Constants.LOOP_PERIOD_SECS);

        inputs.intakeConnected = true;
        inputs.feederConnected = true;
        inputs.hasNote = intakeSim.getGamePiecesAmount() > 0;

        inputs.intakeAppliedVolts = intakeVoltage;
        inputs.intakeStatorCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
        inputs.intakeSupplyCurrentAmps = intakeMotorSim.getCurrentDrawAmps();
        inputs.intakeVelocityRPM = intakeMotorSim.getAngularVelocityRPM();

        inputs.feederAppliedVolts = feederVoltage;
        inputs.feederStatorCurrentAmps = feederMotorSim.getCurrentDrawAmps();
        inputs.feederSupplyCurrentAmps = feederMotorSim.getCurrentDrawAmps();
        inputs.feederVelocityRPM = feederMotorSim.getAngularVelocityRPM();

        // Start intake if intake velocity is greater than 75% of max speed
        boolean running = intakeVoltage >= 6;
        Logger.recordOutput("SimState/Intake/Running", running);
        if (running) intakeSim.startIntake();
        else intakeSim.stopIntake();

        // Accumulate voltage integrals if note is present
        if (inputs.hasNote) intakeVoltageIntegral += inputs.intakeAppliedVolts * Constants.LOOP_PERIOD_SECS;
        else intakeVoltageIntegral = 0.0;

        if (inputs.hasNote) feederVoltageIntegral += inputs.feederAppliedVolts * Constants.LOOP_PERIOD_SECS;
        else feederVoltageIntegral = 0.0;

        // Eject note if intake voltage integral is negative enough
        if (intakeVoltageIntegral <= -12.0 * INTAKE_INTEGRAL_COEFF && intakeSim.obtainGamePieceFromIntake()) {
            RobotState.getInstance().simEjectNote();
        }

        // Shoot note if feeder voltage integral is positive enough
        if (feederVoltageIntegral >= 12.0 * FEED_INTEGRAL_COEFF
                && intakeVoltageIntegral >= 12.0 * INTAKE_INTEGRAL_COEFF
                && intakeSim.obtainGamePieceFromIntake())
            RobotState.getInstance().simShootNote();

        // Amp note if feeder voltage integral is negative enough
        if (feederVoltageIntegral <= -12.0 * FEED_INTEGRAL_COEFF
                && intakeVoltageIntegral >= 12.0 * INTAKE_INTEGRAL_COEFF
                && intakeSim.obtainGamePieceFromIntake())
            RobotState.getInstance().simAmpNote();
    }

    @Override
    public void setIntakeVoltage(double intakeVoltage) {
        this.intakeVoltage = intakeVoltage;
    }

    @Override
    public void setFeederVoltage(double feederVoltage) {
        this.feederVoltage = feederVoltage;
    }
}
