package org.team1540.swervedrive.subsystems.indexer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;

public class Indexer extends SubsystemBase {
    public static final int INTAKE_ID = 10;
    public static final int FEEDER_ID = 11;

    public static final double INTAKE_GEARING = 18.0 / 12.0;
    public static final double FEEDER_GEARING = 18.0 / 12.0;

    public static final double INTAKE_WIDTH_METERS = Units.inchesToMeters(25.716);

    public enum IndexerState {
        STOP(0.0, 0.0),
        INTAKE(1.0, 0.0),
        EJECT(-1.0, -1.0),
        FEED_SHOOTER(1.0, 1.0),
        FEED_AMP(1.0, -1.0);

        public final double intakePercent;
        public final double feederPercent;

        IndexerState(double intakePercent, double feederPercent) {
            this.intakePercent = intakePercent;
            this.feederPercent = feederPercent;
        }
    }

    private static boolean hasInstance = false;

    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    @AutoLogOutput(key = "Indexer/GoalState")
    private IndexerState goalState = IndexerState.STOP;

    private final Alert intakeDisconnectedAlert = new Alert("Intake motor is disconnected", Alert.AlertType.kError);
    private final Alert feederDisconnectedAlert = new Alert("Feeder motor is disconnected", Alert.AlertType.kError);

    private Indexer(IndexerIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of indexer already exists");
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        io.setIntakeVoltage(goalState.intakePercent * 12.0);
        io.setFeederVoltage(goalState.feederPercent * 12.0);

        intakeDisconnectedAlert.set(!inputs.intakeConnected);
        feederDisconnectedAlert.set(!inputs.feederConnected);
    }

    private void setGoal(IndexerState state) {
        goalState = state;
    }

    public void stop() {
        setGoal(IndexerState.STOP);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public Command requestStateCommand(IndexerState state) {
        return Commands.runOnce(() -> setGoal(state), this);
    }

    public Command persistentStateCommand(IndexerState state) {
        return Commands.run(() -> setGoal(state), this).finallyDo(this::stop);
    }

    public static Indexer createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        return new Indexer(new IndexerIO() {});
    }

    public static Indexer createSim(SwerveDriveSimulation driveSim) {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated indexer on real robot", false);
        return new Indexer(new IndexerIOSim(driveSim));
    }

    public static Indexer createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy indexer on real robot", false);
        return new Indexer(new IndexerIO() {});
    }
}
