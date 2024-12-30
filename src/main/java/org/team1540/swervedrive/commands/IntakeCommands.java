package org.team1540.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.swervedrive.subsystems.feeder.Feeder;
import org.team1540.swervedrive.subsystems.intake.Intake;

public class IntakeCommands {
    public static Command continuousIntakeCommand(Intake intake, Feeder feeder) {
        return Commands.repeatingSequence(Commands.either(
                        intake.requestStateCommand(Intake.IntakeState.IDLE)
                                .alongWith(feeder.requestStateCommand(Feeder.FeederState.IDLE)),
                        intake.requestStateCommand(Intake.IntakeState.INTAKE)
                                .alongWith(feeder.requestStateCommand(Feeder.FeederState.FEED)),
                        feeder::hasNote))
                .finallyDo(() -> {
                    intake.stop();
                    feeder.stop();
                });
    }

    public static Command reverseCommand(Intake intake, Feeder feeder) {
        return intake.persistentStateCommand(Intake.IntakeState.REVERSE)
                .alongWith(feeder.persistentStateCommand(Feeder.FeederState.REVERSE));
    }

    public static Command feedCommand(Intake intake, Feeder feeder) {
        return Commands.deadline(
                        Commands.waitUntil(() -> !intake.hasNote() && !feeder.hasNote())
                                .withTimeout(0.5)
                                .andThen(Commands.waitSeconds(0.2)),
                        intake.persistentStateCommand(Intake.IntakeState.INTAKE),
                        feeder.persistentStateCommand(Feeder.FeederState.FEED))
                .onlyIf(() -> intake.hasNote() || feeder.hasNote());
    }
}
