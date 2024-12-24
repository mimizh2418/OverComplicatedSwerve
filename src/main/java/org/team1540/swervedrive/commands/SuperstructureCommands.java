package org.team1540.swervedrive.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.subsystems.arm.Arm;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.subsystems.shooter.Shooter;
import org.team1540.swervedrive.util.AllianceFlipUtil;

public class SuperstructureCommands {
    public static Command teleopSpeakerAimCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, Shooter shooter, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(
                                driver,
                                () -> RobotState.getInstance()
                                        .getSpeakerAimingParameters()
                                        .driveHeading(),
                                isFieldRelative),
                        arm.requestStateCommand(Arm.ArmState.SPEAKER),
                        shooter.persistentStateCommand(Shooter.ShooterState.SPEAKER))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }

    public static Command teleopPassingAimCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, Shooter shooter, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(
                                driver,
                                () -> RobotState.getInstance()
                                        .getPassingAimingParameters()
                                        .driveHeading(),
                                isFieldRelative),
                        arm.requestStateCommand(Arm.ArmState.PASS),
                        shooter.persistentStateCommand(Shooter.ShooterState.PASS))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }

    public static Command teleopDynamicAimCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, Shooter shooter, BooleanSupplier isFieldRelative) {
        var robotState = RobotState.getInstance();
        return Commands.deadline(
                        drivetrain.teleopDriveWithHeadingCommand(
                                driver,
                                () -> FieldConstants.inWing(robotState.getRobotPose())
                                        ? robotState
                                                .getSpeakerAimingParameters()
                                                .driveHeading()
                                        : robotState
                                                .getPassingAimingParameters()
                                                .driveHeading(),
                                isFieldRelative),
                        Commands.repeatingSequence(Commands.either(
                                arm.requestStateCommand(Arm.ArmState.SPEAKER),
                                arm.requestStateCommand(Arm.ArmState.PASS),
                                () -> FieldConstants.inWing(robotState.getRobotPose()))),
                        Commands.repeatingSequence(Commands.either(
                                shooter.requestStateCommand(Shooter.ShooterState.SPEAKER),
                                shooter.requestStateCommand(Shooter.ShooterState.PASS),
                                () -> FieldConstants.inWing(robotState.getRobotPose()))))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)))
                .finallyDo(shooter::stop);
    }

    public static Command teleopStageAmpCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(
                                driver,
                                () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCCW_90deg),
                                isFieldRelative),
                        arm.requestStateCommand(Arm.ArmState.AMP))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }
}
