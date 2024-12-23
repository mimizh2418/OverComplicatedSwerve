package org.team1540.swervedrive.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.util.ClosedLoopConfig;
import org.team1540.swervedrive.util.LoggedTunableNumber;

import java.util.function.Supplier;

public class Arm extends SubsystemBase {
    public static int LEADER_ID = 20;
    public static int FOLLOWER_ID = 21;
    public static int ENCODER_ID = 23;

    public static final double GEAR_RATIO = (62.0 / 12.0) * (60.0 / 18.0) * (65.0 / 12.0);
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(3.0);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(5.8);
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(110.0);

    public static final ClosedLoopConfig GAINS =
            new ClosedLoopConfig(new PIDController(90.0, 0.0, 0.0), new ArmFeedforward(0.0, 0.0, 0.0));

    public static final Translation2d ORIGIN = new Translation2d(-0.238, 0.298);

    public static final Supplier<Rotation2d> STOW = () -> MIN_ANGLE;
    public static final Supplier<Rotation2d> AMP = () -> Rotation2d.fromDegrees(100.0);

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", GAINS.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", GAINS.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", GAINS.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", GAINS.kS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", GAINS.kV);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", GAINS.kG);

    private static boolean hasInstance = false;

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private Supplier<Rotation2d> setpoint = STOW;

    private final Alert leaderDisconnectedAlert = new Alert("Arm leader motor disconnected", Alert.AlertType.kError);
    private final Alert follwerDisconnectedAlert = new Alert("Arm leader follower disconnected", Alert.AlertType.kError);
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

        if (DriverStation.isDisabled()) stop();
        else {
            Rotation2d currentSetpoint = setpoint.get();
            io.setPosition(currentSetpoint);
            Logger.recordOutput("Arm/setpoint", currentSetpoint);
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get(), kG.get()), kS, kV, kG);
    }

    public void stop() {
        io.setVoltage(0);
    }

    public boolean atGoal() {
        return MathUtil.isNear(setpoint.get().getDegrees(), inputs.position.getDegrees(), TOLERANCE.getDegrees());
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    private void runSetpoint(Supplier<Rotation2d> setpoint) {
        this.setpoint = setpoint;
    }

    /** Returns a non-blocking command that sets the arm setpoint and ends */
    public Command setpointCommand(Supplier<Rotation2d> setpoint) {
        return Commands.runOnce(() -> runSetpoint(setpoint), this);
    }

    /** Returns a blocking command that sets the arm setpoint and waits until the arm is at the setpoint */
    public Command blockingSetpointCommand(Supplier<Rotation2d> setpoint) {
        return setpointCommand(setpoint).andThen(Commands.waitUntil(this::atGoal));
    }
}
