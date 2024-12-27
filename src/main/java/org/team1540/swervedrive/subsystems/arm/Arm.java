package org.team1540.swervedrive.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
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
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.CharacterizationCommands;
import org.team1540.swervedrive.util.ClosedLoopConfig;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class Arm extends SubsystemBase {
    public static int LEADER_ID = 20;
    public static int FOLLOWER_ID = 21;
    public static int ENCODER_ID = 23;

    public static final double GEAR_RATIO = (62.0 / 12.0) * (60.0 / 18.0) * (65.0 / 12.0);
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3.0);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(5.8);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(110.0);

    public static final Translation3d PIVOT_ORIGIN = new Translation3d(-0.238, 0, 0.298);
    public static final double LENGTH_METERS = Units.inchesToMeters(25.866);

    public static final double ENCODER_OFFSET_ROTS = 0.5;

    public static final ClosedLoopConfig GAINS = new ClosedLoopConfig(
            45.0,
            0.0,
            0.0,
            0.0,
            Constants.currentMode == Constants.Mode.SIM ? 1.83527 : 11.53186,
            0.0,
            0.0,
            ClosedLoopConfig.GravityFFType.ARM);
    public static final double MAX_VELOCITY_RPS = 1.0;
    public static final double MAX_ACCELERATION_RPS2 = 2.54;

    public enum ArmState {
        STOW(() -> MIN_ANGLE),
        AMP(() -> Rotation2d.fromDegrees(100.0)),
        SUBWOOFER(() -> Rotation2d.fromDegrees(55.0)),
        PODIUM(() -> Rotation2d.fromDegrees(34.0)),
        SPEAKER(() -> RobotState.getInstance().getSpeakerAimingParameters().armAngle()),
        PASS(() -> RobotState.getInstance().getPassingAimingParameters().armAngle()),
        LOW_PASS(() -> RobotState.getInstance().getLowPassingAimingParameters().armAngle());

        public final Supplier<Rotation2d> angleSupplier;

        ArmState(Supplier<Rotation2d> angleSupplier) {
            this.angleSupplier = angleSupplier;
        }
    }

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", GAINS.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", GAINS.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", GAINS.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", GAINS.kS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", GAINS.kV);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", GAINS.kG);

    private static boolean hasInstance = false;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    @AutoLogOutput(key = "Arm/GoalState")
    private ArmState goalState = ArmState.STOW;

    private boolean isCharacterizing = false;

    private final Alert leaderDisconnectedAlert = new Alert("Arm leader motor disconnected", Alert.AlertType.kError);
    private final Alert follwerDisconnectedAlert =
            new Alert("Arm leader follower disconnected", Alert.AlertType.kError);
    private final Alert encoderDisconnectedAlert = new Alert("Arm encoder disconnected", Alert.AlertType.kError);

    private Arm(ArmIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of Arm already exists");
        this.io = io;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        Rotation2d currentSetpoint = goalState.angleSupplier.get();
        Logger.recordOutput("Arm/Setpoint", currentSetpoint);

        if (DriverStation.isDisabled()) stop();
        else if (!isCharacterizing) io.setPosition(currentSetpoint);

        RobotState.getInstance().addArmAngleData(inputs.position, goalState.angleSupplier.get());

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);

        leaderDisconnectedAlert.set(!inputs.leaderConnected);
        follwerDisconnectedAlert.set(!inputs.followerConnected);
        encoderDisconnectedAlert.set(!inputs.encoderConnected);
    }

    public void stop() {
        io.setVoltage(0);
    }

    private void setGoal(ArmState goalState) {
        this.goalState = goalState;
    }

    public boolean atGoal() {
        return MathUtil.isNear(
                goalState.angleSupplier.get().getDegrees(), inputs.position.getDegrees(), TOLERANCE.getDegrees());
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    /** Returns a non-blocking command that sets the arm setpoint and ends */
    public Command requestStateCommand(ArmState state) {
        return Commands.runOnce(() -> setGoal(state), this);
    }

    public Command persistentStateCommand(ArmState state) {
        return Commands.run(() -> setGoal(state), this).finallyDo(this::stop);
    }

    /** Returns a blocking command that sets the arm state and waits for the arm to reach that state.*/
    public Command blockingStateCommand(ArmState state) {
        return requestStateCommand(state).andThen(Commands.waitUntil(this::atGoal));
    }

    public Command feedforwardCharacterization() {
        return CharacterizationCommands.feedforward(io::setVoltage, () -> inputs.velocityRPS, this)
                .beforeStarting(() -> isCharacterizing = true)
                .finallyDo(() -> isCharacterizing = false);
    }

    public static Arm createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real arm on simulated robot", false);
        }
        return new Arm(new ArmIOTalonFX());
    }

    public static Arm createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated arm on real robot", false);
        }
        return new Arm(new ArmIOSim());
    }

    public static Arm createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy arm on real robot", false);
        }
        return new Arm(new ArmIO() {});
    }
}
