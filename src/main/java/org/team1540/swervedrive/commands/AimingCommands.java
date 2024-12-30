package org.team1540.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.subsystems.pivot.Pivot;
import org.team1540.swervedrive.subsystems.shooter.Shooter;
import org.team1540.swervedrive.subsystems.turret.Turret;

public class AimingCommands {
    public static Command speakerAimCommand(Turret turret, Pivot pivot, Shooter shooter) {
        return Commands.parallel(
                turret.persistentStateCommand(Turret.TurretState.SPEAKER),
                pivot.persistentStateCommand(Pivot.PivotState.SPEAKER),
                shooter.persistentStateCommand(Shooter.ShooterState.SPEAKER));
    }

    public static Command passAimCommand(Turret turret, Pivot pivot, Shooter shooter) {
        return Commands.parallel(
                turret.persistentStateCommand(Turret.TurretState.PASS),
                pivot.persistentStateCommand(Pivot.PivotState.PASS),
                shooter.persistentStateCommand(Shooter.ShooterState.PASS));
    }

    public static Command dynamicAimCommand(Turret turret, Pivot pivot, Shooter shooter) {
        return Commands.parallel(
                        turret.dynamicTrackingCommand(),
                        pivot.dynamicAimCommand(),
                        Commands.repeatingSequence(Commands.either(
                                shooter.requestStateCommand(Shooter.ShooterState.SPEAKER),
                                shooter.requestStateCommand(Shooter.ShooterState.PASS),
                                () -> FieldConstants.inOwnWing(
                                        RobotState.getInstance().getRobotPose()))))
                .finallyDo(shooter::stop);
    }
}
