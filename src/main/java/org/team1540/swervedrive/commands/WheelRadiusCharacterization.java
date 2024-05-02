package org.team1540.swervedrive.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

// NOTE: This file is available at
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/commands/FeedForwardCharacterization.java

public class WheelRadiusCharacterization extends Command {
    private static final LoggedTunableNumber characterizationSpeed =
            new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
    private static final double driveRadius = Drivetrain.DRIVE_BASE_RADIUS;
    private static final DoubleSupplier gyroYawRadsSupplier =
            () -> RobotState.getInstance().getRawGyroRotation().getRadians();

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;
        Direction(int value) {
            this.value = value;
        }
    }

    private final Drivetrain drivetrain;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(Drivetrain drivetrain, Direction omegaDirection) {
        this.drivetrain = drivetrain;
        this.omegaDirection = omegaDirection;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = drivetrain.getWheelRadiusCharacterizationPositions();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        drivetrain.runWheelRadiusCharacterization(
                omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositions = drivetrain.getWheelRadiusCharacterizationPositions();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
        Logger.recordOutput("CharacterizationResults/WheelRadius/DrivePosition", averageWheelPosition);
        Logger.recordOutput("CharacterizationResults/WheelRadius/AccumGyroYawRads", accumGyroYawRads);
        Logger.recordOutput(
                "CharacterizationResults/WheelRadius/CurrentWheelRadiusInches",
                Units.metersToInches(currentEffectiveWheelRadius));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.endCharacterization();
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
            Logger.recordOutput(
                    "CharacterizationResults/WheelRadius/EffectiveWheelRadiusInches",
                    Units.metersToInches(currentEffectiveWheelRadius));
        }
    }
}
