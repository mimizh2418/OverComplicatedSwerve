package org.team1540.swervedrive.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.util.math.PolynomialRegression;

// NOTE: This file is available at
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/commands/FeedForwardCharacterization.java

public class FeedForwardCharacterization extends Command {
    private static final double START_DELAY_SECS = 2.0;
    private static final double RAMP_VOLTS_PER_SEC = 0.1;

    private FeedForwardCharacterizationData data;
    private final Consumer<Double> voltageConsumer;
    private final Supplier<Double> velocitySupplier;

    private final Timer timer = new Timer();

    /** Creates a new FeedForwardCharacterization command. */
    public FeedForwardCharacterization(
            Subsystem subsystem,
            Consumer<Double> voltageConsumer,
            Supplier<Double> velocitySupplier) {
        addRequirements(subsystem);
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        data = new FeedForwardCharacterizationData();
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < START_DELAY_SECS) {
            voltageConsumer.accept(0.0);
        } else {
            double voltage = (timer.get() - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC;
            voltageConsumer.accept(voltage);
            data.add(velocitySupplier.get(), voltage);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(0.0);
        timer.stop();
        data.printAndLog();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public static class FeedForwardCharacterizationData {
        private final List<Double> velocityData = new LinkedList<>();
        private final List<Double> voltageData = new LinkedList<>();

        public void add(double velocity, double voltage) {
            if (Math.abs(velocity) > 1E-4) {
                velocityData.add(Math.abs(velocity));
                voltageData.add(Math.abs(voltage));
            }
        }

        public void printAndLog() {
            if (velocityData.isEmpty() || voltageData.isEmpty()) {
                return;
            }

            PolynomialRegression regression =
                    new PolynomialRegression(
                            velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
                            voltageData.stream().mapToDouble(Double::doubleValue).toArray(),
                            1);

            System.out.println("FF Characterization Results:");
            System.out.println("\tCount=" + velocityData.size());
            System.out.printf("\tR2=%.5f%n", regression.R2());
            System.out.printf("\tkS=%.5f%n", regression.beta(0));
            System.out.printf("\tkV=%.5f%n", regression.beta(1));

            Logger.recordOutput("CharacterizationResults/Feedforward/Count", velocityData.size());
            Logger.recordOutput("CharacterizationResults/Feedforward/R2", regression.R2());
            Logger.recordOutput("CharacterizationResults/Feedforward/kS", regression.beta(0));
            Logger.recordOutput("CharacterizationResults/Feedforward/kV", regression.beta(1));
        }
    }
}
