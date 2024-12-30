package org.team1540.swervedrive.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.CharacterizationCommands;
import org.team1540.swervedrive.util.ClosedLoopConfig;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class Pivot extends SubsystemBase {
    public static final double GEAR_RATIO = 60.95;

    public static final Rotation2d MIN_ANGLE = Rotation2d.kZero;
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(70.0);
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);

    public static final double LENGTH_METERS = Units.inchesToMeters(12.0);
    public static final Translation3d TURRET_TO_PIVOT_METERS =
            new Translation3d(Units.inchesToMeters(-3.544506), 0.0, Units.inchesToMeters(11.295764));

    public static final ClosedLoopConfig GAINS = new ClosedLoopConfig(50.0, 0.0, 10.0, 0.00231, 7.53428, 0.0);
    public static final double MAX_VELOCITY_RPS = 2.54;
    public static final double MAX_ACCELERATION_RPS2 = 6.328;

    public enum PivotState {
        STOW(() -> MIN_ANGLE),
        SPEAKER(() -> RobotState.getInstance().getSpeakerAimingParameters().pivotAngle()),
        PASS(() -> RobotState.getInstance().getPassingAimingParameters().pivotAngle());

        private final Supplier<Rotation2d> position;

        PivotState(Supplier<Rotation2d> position) {
            this.position = position;
        }
    }

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", GAINS.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/kI", GAINS.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", GAINS.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", GAINS.kS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV", GAINS.kV);

    private static boolean hasInstance = false;

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    @AutoLogOutput(key = "Pivot/GoalState")
    private PivotState goalState = PivotState.STOW;

    private boolean isCharacterizing = false;

    private final Alert disconnectedAlert = new Alert("Pivot motor is disconnected", Alert.AlertType.kError);

    private Pivot(PivotIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of pivot already exists");
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        Rotation2d currentSetpoint = goalState.position.get();
        Logger.recordOutput("Pivot/Setpoint", currentSetpoint);

        if (DriverStation.isDisabled()) stop();
        else if (!isCharacterizing) io.setPosition(currentSetpoint);

        RobotState.getInstance().addPivotAngleData(inputs.position);

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get()), kS, kV);

        disconnectedAlert.set(!inputs.connected);
    }

    private void setGoal(PivotState state) {
        goalState = state;
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    public boolean atGoal() {
        return MathUtil.isNear(
                goalState.position.get().getDegrees(), inputs.position.getDegrees(), TOLERANCE.getDegrees());
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    public Command requestStateCommand(PivotState state) {
        return Commands.runOnce(() -> setGoal(state), this);
    }

    public Command persistentStateCommand(PivotState state) {
        return Commands.run(() -> setGoal(state), this).finallyDo(this::stop);
    }

    public Command blockingStateCommand(PivotState state) {
        return requestStateCommand(state).andThen(Commands.waitUntil(this::atGoal));
    }

    public Command dynamicAimCommand() {
        return Commands.repeatingSequence(Commands.either(
                        requestStateCommand(PivotState.SPEAKER),
                        requestStateCommand(PivotState.PASS),
                        () -> FieldConstants.inOwnWing(RobotState.getInstance().getRobotPose())))
                .finallyDo(() -> setGoal(PivotState.STOW));
    }

    public Command feedforwardCharacterization() {
        return CharacterizationCommands.feedforward(io::setVoltage, () -> inputs.velocityRPS, this)
                .beforeStarting(() -> isCharacterizing = true)
                .finallyDo(() -> isCharacterizing = false);
    }

    public static Pivot createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real pivot on simulated robot", false);
        return new Pivot(new PivotIO() {});
    }

    public static Pivot createSim() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated pivot on real robot", false);
        return new Pivot(new PivotIOSim());
    }

    public static Pivot createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy pivot on real robot", false);
        return new Pivot(new PivotIO() {});
    }
}
