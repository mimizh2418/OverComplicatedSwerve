package org.team1540.swervedrive.subsystems.turret;

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
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.util.ClosedLoopConfig;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class Turret extends SubsystemBase {
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(360.0);
    public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-360.0);
    public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2.0);

    public static final double GEAR_RATIO = 54.44;

    public static final ClosedLoopConfig GAINS = new ClosedLoopConfig(254, 0.0, 0.0, 0.00207, 1.07110);
    public static final double MAX_VELOCITY_RPS =
            2 * Units.radiansToRotations(Drivetrain.MAX_ANGULAR_SPEED_RADS_PER_SEC);
    public static final double MAX_ACCELERATION_RPS2 = MAX_VELOCITY_RPS * 4.0;

    public static final Translation3d ORIGIN_METERS = new Translation3d(Units.inchesToMeters(-4.0), 0.0, 0.0);

    public enum TurretState {
        STOW(() -> Rotation2d.kZero),
        SPEAKER(() -> RobotState.getInstance().getSpeakerAimingParameters().turretAngle()),
        PASS(() -> RobotState.getInstance().getPassingAimingParameters().turretAngle()),
        AMP(() -> Rotation2d.k180deg);

        private final Supplier<Rotation2d> position;

        TurretState(Supplier<Rotation2d> position) {
            this.position = position;
        }
    }

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP", GAINS.kP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/kI", GAINS.kI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD", GAINS.kD);
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/kS", GAINS.kS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/kV", GAINS.kV);

    private static boolean hasInstance = false;

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    @AutoLogOutput(key = "Turret/GoalState")
    private TurretState goalState = TurretState.STOW;

    private boolean isCharacterizing = false;

    private final Alert disconnectedAlert = new Alert("Turret motor disconnected", Alert.AlertType.kError);

    private Turret(TurretIO turretIO) {
        if (hasInstance) throw new IllegalStateException("Instance of turret already exists");
        this.io = turretIO;
        hasInstance = true;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Rotation2d currentPositionSetpoint = goalState.position.get();
        Logger.recordOutput("Turret/PositionSetpoint", currentPositionSetpoint);

        if (DriverStation.isDisabled()) stop();
        else if (!isCharacterizing) io.setPosition(optimizePositionSetpoint(currentPositionSetpoint));

        RobotState.getInstance().addTurretAngleData(inputs.position, inputs.velocityRPS, inputs.timestamp);

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setFF(kS.get(), kV.get()), kS, kV);

        disconnectedAlert.set(!inputs.connected);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    private void setGoal(TurretState state) {
        goalState = state;
    }

    public boolean atGoal() {
        return Math.abs(goalState.position.get().minus(inputs.position).getDegrees()) < TOLERANCE.getDegrees();
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    private Rotation2d optimizePositionSetpoint(Rotation2d setpoint) {
        double currentPositionDegrees = inputs.position.getDegrees();

        double boundedSetpointDegrees = Math.toDegrees(MathUtil.angleModulus(setpoint.getRadians()));
        double ccwSetpointDegrees = boundedSetpointDegrees + 360.0;
        double cwSetpointDegrees = boundedSetpointDegrees - 360.0;

        double closestSetpointDegrees = boundedSetpointDegrees;
        double closestDistance = Math.abs(boundedSetpointDegrees - currentPositionDegrees);

        double ccwDistance = Math.abs(ccwSetpointDegrees - currentPositionDegrees);
        if (ccwDistance < closestDistance) {
            closestSetpointDegrees = ccwSetpointDegrees;
            closestDistance = ccwDistance;
        }

        double cwDistance = Math.abs(cwSetpointDegrees - currentPositionDegrees);
        if (cwDistance < closestDistance) {
            closestSetpointDegrees = cwSetpointDegrees;
        }

        if (closestSetpointDegrees <= 360 && closestSetpointDegrees >= -360) {
            return Rotation2d.fromDegrees(closestSetpointDegrees);
        } else {
            return Rotation2d.fromDegrees(boundedSetpointDegrees);
        }
    }

    public Command requestStateCommand(TurretState state) {
        return Commands.runOnce(() -> setGoal(state), this);
    }

    public Command persistentStateCommand(TurretState state) {
        return Commands.run(() -> setGoal(state), this).finallyDo(this::stop);
    }

    public Command dynamicTrackingCommand() {
        return Commands.repeatingSequence(Commands.either(
                        requestStateCommand(TurretState.SPEAKER),
                        requestStateCommand(TurretState.PASS),
                        () -> FieldConstants.inOwnWing(RobotState.getInstance().getRobotPose())))
                .finallyDo(() -> setGoal(TurretState.STOW));
    }

    public Command feedforwardCharacterization() {
        return CharacterizationCommands.feedforward(io::setVoltage, () -> inputs.velocityRPS, this)
                .beforeStarting(() -> isCharacterizing = true)
                .finallyDo(() -> isCharacterizing = false);
    }

    public static Turret createReal() {
        if (Constants.currentMode != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real turret on simulated robot", false);
        return new Turret(new TurretIO() {});
    }

    public static Turret createSim() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using simulated turret on real robot", false);
        return new Turret(new TurretIOSim());
    }

    public static Turret createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy turret on real robot", false);
        return new Turret(new TurretIO() {});
    }
}
