package org.team1540.swervedrive.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;

public class Intake extends SubsystemBase {
    public static final double GEAR_RATIO = 20.0 / 12.0;

    public static final double WIDTH_METERS = Units.inchesToMeters(22.796);
    public static final double EXTENSION_METERS = Units.inchesToMeters(4.25);

    public enum IntakeState {
        IDLE(0.0, 0.0),
        INTAKE(12.0, 12.0),
        REVERSE(-12.0, -12.0);

        public final double topVoltage;
        public final double bottomVoltage;

        IntakeState(double topVoltage, double bottomVoltage) {
            this.topVoltage = topVoltage;
            this.bottomVoltage = bottomVoltage;
        }
    }

    private static boolean hasInstance = false;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    @AutoLogOutput(key = "Intake/GoalState")
    private IntakeState goalState = IntakeState.IDLE;

    private final Alert topDisconnectedAlert = new Alert("Top intake motor disconnected", Alert.AlertType.kError);
    private final Alert bottomDisconnectedAlert = new Alert("Bottom intake motor disconnected", Alert.AlertType.kError);

    private Intake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        io.setVoltage(goalState.topVoltage, goalState.bottomVoltage);

        topDisconnectedAlert.set(!inputs.topConnected);
        bottomDisconnectedAlert.set(!inputs.bottomConnected);
    }

    private void setGoalState(IntakeState state) {
        goalState = state;
    }

    public void stop() {
        setGoalState(IntakeState.IDLE);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public Command requestStateCommand(IntakeState state) {
        return Commands.runOnce(() -> setGoalState(state), this);
    }

    public Command persistentStateCommand(IntakeState state) {
        return Commands.run(() -> setGoalState(state), this).finallyDo(this::stop);
    }

    public static Intake createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        return new Intake(new IntakeIO() {});
    }

    public static Intake createSim() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated indexer on real robot", false);
        return new Intake(new IntakeIOSim());
    }

    public static Intake createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy indexer on real robot", false);
        return new Intake(new IntakeIO() {});
    }
}
