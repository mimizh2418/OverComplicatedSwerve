package org.team1540.swervedrive.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.util.LoggedTunableNumber;

public class Module {
    public static final LoggedTunableNumber driveKP =
            new LoggedTunableNumber("Drivetrain/Modules/DriveKP", Drivetrain.MODULE_CONFIG.driveVelocityGains().kP);
    public static final LoggedTunableNumber driveKD =
            new LoggedTunableNumber("Drivetrain/Modules/DriveKD", Drivetrain.MODULE_CONFIG.driveVelocityGains().kD);
    public static final LoggedTunableNumber driveKS =
            new LoggedTunableNumber("Drivetrain/Modules/DriveKS", Drivetrain.MODULE_CONFIG.driveVelocityGains().kS);
    public static final LoggedTunableNumber driveKV =
            new LoggedTunableNumber("Drivetrain/Modules/DriveKV", Drivetrain.MODULE_CONFIG.driveVelocityGains().kV);
    public static final LoggedTunableNumber turnKP =
            new LoggedTunableNumber("Drivetrain/Modules/TurnKP", Drivetrain.MODULE_CONFIG.turnPositionGains().kP);
    public static final LoggedTunableNumber turnKD =
            new LoggedTunableNumber("Drivetrain/Modules/TurnKD", Drivetrain.MODULE_CONFIG.turnPositionGains().kD);

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
        setBrakeMode(true);
    }

    /** Update inputs without running the rest of the periodic logic. This is useful since these
     * updates need to be properly thread-locked. */
    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void periodic() {
        Logger.processInputs("Drivetrain/Module" + index, inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionsRads[i] * Drivetrain.WHEEL_RADIUS;
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> io.setDriveFF(driveKS.get(), driveKV.get()), driveKS, driveKV);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setDrivePID(driveKP.get(), 0.0, driveKD.get()), driveKP, driveKD);
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setTurnPID(turnKP.get(), 0.0, turnKD.get()), turnKP, turnKD);
    }

    /** Runs the module with the specified setpoint state. Returns the optimized state. */
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        SwerveModuleState setpointState = SwerveModuleState.optimize(state, getTurnAngle());
        io.setDriveVelocity(setpointState.speedMetersPerSecond / Drivetrain.WHEEL_RADIUS);
        io.setTurnPosition(setpointState.angle);

        return setpointState;
    }

    /** Runs the module with the specified voltage while controlling to zero degrees.*/
    public void runCharacterization(double volts) {
        // Closed loop turn control
        io.setTurnPosition(new Rotation2d());

        // Open loop drive control
        io.setDriveVoltage(volts);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getTurnAngle() {
        return inputs.turnPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getDrivePositionMeters() {
        return inputs.drivePositionRads * Drivetrain.WHEEL_RADIUS;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getDriveVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * Drivetrain.WHEEL_RADIUS;
    }

    /** Returns the module position (turn a */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getTurnAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getTurnAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /** Returns the drive velocity in rotations/sec. */
    public double getCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }

    /** Returns the drive position in radians. */
    public double getCharacterizationPosition() {
        return inputs.drivePositionRads;
    }
}
