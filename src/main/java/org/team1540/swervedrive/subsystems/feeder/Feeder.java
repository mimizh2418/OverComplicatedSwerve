package org.team1540.swervedrive.subsystems.feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;

public class Feeder extends SubsystemBase {
    public static final double TEACUP_GEAR_RATIO = 2.34;
    public static final double FEEDER_GEAR_RATIO = 10.0;

    public enum FeederState {
        IDLE(0.0, 0.0),
        FEED(12.0, 12.0),
        REVERSE(-12.0, -12.0);

        public final double teacupVoltage;
        public final double feederVoltage;

        FeederState(double teacupVoltage, double feederVoltage) {
            this.teacupVoltage = teacupVoltage;
            this.feederVoltage = feederVoltage;
        }
    }

    private static boolean hasInstance = false;

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    @AutoLogOutput(key = "Intake/GoalState")
    private FeederState goalState = FeederState.IDLE;

    private final Alert teacupDisconnectedAlert = new Alert("Teacup motor disconnected", Alert.AlertType.kError);
    private final Alert feederLeadDisconnectedAlert =
            new Alert("Feeder lead motor disconnected", Alert.AlertType.kError);
    private final Alert feederFollowDisconnectedAlert =
            new Alert("Feeder follow motor disconnected", Alert.AlertType.kError);

    private Feeder(FeederIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of feeder already exists");
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        io.setVoltage(goalState.teacupVoltage, goalState.feederVoltage);

        teacupDisconnectedAlert.set(!inputs.teacupConnected);
        feederLeadDisconnectedAlert.set(!inputs.feederLeadConnected);
        feederFollowDisconnectedAlert.set(!inputs.feederFollowConnected);
    }

    private void setGoalState(FeederState state) {
        goalState = state;
    }

    public void stop() {
        setGoalState(FeederState.IDLE);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public Command requestStateCommand(FeederState state) {
        return Commands.runOnce(() -> setGoalState(state), this);
    }

    public Command persistentStateCommand(FeederState state) {
        return Commands.run(() -> setGoalState(state), this).finallyDo(this::stop);
    }

    public static Feeder createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real feeder on simulated robot", false);
        return new Feeder(new FeederIO() {});
    }

    public static Feeder createSim() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated feeder on real robot", false);
        return new Feeder(new FeederIOSim());
    }

    public static Feeder createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy feeder on real robot", false);
        return new Feeder(new FeederIO() {});
    }
}
