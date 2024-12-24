package org.team1540.swervedrive.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.CharacterizationCommands;
import org.team1540.swervedrive.util.ClosedLoopConfig;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    public static final int LEFT_ID = 30;
    public static final int RIGHT_ID = 31;

    public static final double GEARING = 18.0 / 24.0;

    public static final ClosedLoopConfig GAINS = new ClosedLoopConfig(0.0005, 0.0, 0.0, 0.00931, 0.06953, 0.0);

    public enum ShooterState {
        IDLE(0.0, 0.0),
        SPEAKER(5066.0, 7733.0),
        PASS(3000.0, 3000.0);

        public final double leftVelocityRPM;
        public final double rightVelocityRPM;

        ShooterState(double leftVelocityRPM, double rightVelocityRPM) {
            this.leftVelocityRPM = leftVelocityRPM;
            this.rightVelocityRPM = rightVelocityRPM;
        }
    }

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", GAINS.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", GAINS.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", GAINS.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", GAINS.kS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", GAINS.kV);

    private static boolean hasInstance = false;

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final Alert leftDisconnectedAlert = new Alert("Left shooter motor is disconnected", Alert.AlertType.kError);
    private final Alert rightDisconnectedAlert =
            new Alert("Right shooter motor is disconnected", Alert.AlertType.kError);

    @AutoLogOutput(key = "Shooter/GoalState")
    private ShooterState goalState = ShooterState.IDLE;

    @AutoLogOutput
    private boolean isCharacterizing = false;

    private final LinearFilter leftVelocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private final LinearFilter rightVelocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private Shooter(ShooterIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/LeftFilteredVelocity", leftVelocityFilter.calculate(inputs.leftVelocityRPM));
        Logger.recordOutput("Shooter/RightFilteredVelocity", rightVelocityFilter.calculate(inputs.rightVelocityRPM));

        RobotState.getInstance().addShooterVelocityData(inputs.leftVelocityRPM, inputs.rightVelocityRPM);

        if (DriverStation.isDisabled()) stop();
        else if (!isCharacterizing) {
            if (goalState == ShooterState.IDLE) io.setVoltage(0.0, 0.0);
            else io.setVelocity(goalState.leftVelocityRPM, goalState.rightVelocityRPM);
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get()), kS, kV);

        leftDisconnectedAlert.set(!inputs.leftConnected);
        rightDisconnectedAlert.set(!inputs.rightConnected);
    }

    private void setGoalState(ShooterState goalState) {
        this.goalState = goalState;
    }

    @AutoLogOutput(key = "Shooter/AtGoal")
    public boolean atGoal() {
        return MathUtil.isNear(goalState.leftVelocityRPM, leftVelocityFilter.lastValue(), 500.0)
                && MathUtil.isNear(goalState.rightVelocityRPM, rightVelocityFilter.lastValue(), 500.0);
    }

    public void stop() {
        setGoalState(ShooterState.IDLE);
        io.setVoltage(0.0, 0.0);
    }

    public Command requestStateCommand(ShooterState state) {
        return Commands.runOnce(() -> setGoalState(state), this);
    }

    public Command persistentStateCommand(ShooterState state) {
        return requestStateCommand(state).andThen(Commands.idle(this)).finallyDo(this::stop);
    }

    public Command blockingStateCommand(ShooterState state) {
        return requestStateCommand(state).andThen(Commands.waitUntil(this::atGoal));
    }

    public Command feedforwardCharacterization() {
        return CharacterizationCommands.feedforward(
                        volts -> io.setVoltage(volts, volts),
                        () -> (inputs.rightVelocityRPM + inputs.leftVelocityRPM) / 2.0 / 60.0,
                        this)
                .beforeStarting(() -> isCharacterizing = true)
                .finallyDo(() -> isCharacterizing = false);
    }

    public static Shooter createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real shooter on simulated robot", false);
        return new Shooter(new ShooterIO() {});
    }

    public static Shooter createSim() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated shooter on real robot", false);
        return new Shooter(new ShooterIOSim());
    }

    public static Shooter createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy shooter on real robot", false);
        return new Shooter(new ShooterIO() {});
    }
}
