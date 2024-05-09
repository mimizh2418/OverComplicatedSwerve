package org.team1540.swervedrive.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveMotorConnected = true;
        public double drivePositionRads = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        public boolean turnMotorConnected = true;
        public boolean turnEncoderConnected = true;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;

        public double[] odometryTimestamps = new double[]{};
        public double[] odometryDrivePositionsRads = new double[]{};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[]{};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ModuleIOInputs inputs) {}

    /** Runs the drive motor in closed-loop at the specified velocity */
    default void setDriveVelocity(double velocityRadPerSec) {}

    /** Run the drive motor at the specified voltage. */
    default void setDriveVoltage(double volts) {}

    /** Runs the turn motor in closed-loop to the specified position */
    default void setTurnPosition(Rotation2d position) {}

    /** Run the turn motor at the specified voltage. */
    default void setTurnVoltage(double volts) {}

    /** Enable or disable brake mode on the drive motor. */
    default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    default void setTurnBrakeMode(boolean enable) {}

    /** Set the feedforward constants for the drive motor. */
    default void setDriveFF(double kS, double kV) {}

    /** Set the PID constants for the drive motor. */
    default void setDrivePID(double kP, double kI, double kD) {}

    /** Set the PID constants for the turn motor. */
    default void setTurnPID(double kP, double kI, double kD) {}
}
