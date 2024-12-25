package org.team1540.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.swervedrive.subsystems.arm.Arm;
import org.team1540.swervedrive.subsystems.shooter.Shooter;

public class SuperstructureCommands {
    public static Command speakerAimCommand(Arm arm, Shooter shooter) {
        return Commands.parallel(
                        arm.requestStateCommand(Arm.ArmState.SPEAKER),
                        shooter.persistentStateCommand(Shooter.ShooterState.SPEAKER))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }

    public static Command passingAimCommand(Arm arm, Shooter shooter) {
        return Commands.parallel(
                        arm.requestStateCommand(Arm.ArmState.PASS),
                        shooter.persistentStateCommand(Shooter.ShooterState.PASS))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }

    public static Command lowPassingAimCommand(Arm arm, Shooter shooter) {
        return Commands.parallel(
                        arm.requestStateCommand(Arm.ArmState.LOW_PASS),
                        shooter.persistentStateCommand(Shooter.ShooterState.LOW_PASS))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }
}
