package org.team1540.swervedrive.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.subsystems.arm.Arm;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.util.AllianceFlipUtil;

public class ScoringCommands {
    public static Command teleopSpeakerAimCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(
                                driver,
                                () -> RobotState.getInstance()
                                        .getSpeakerAimingParameters()
                                        .driveHeading(),
                                isFieldRelative),
                        arm.requestStateCommand(Arm.ArmState.SPEAKER))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }

    public static Command teleopAmpStageCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(
                                driver, () -> AllianceFlipUtil.reverseRotation(Rotation2d.kCCW_90deg), isFieldRelative),
                        arm.requestStateCommand(Arm.ArmState.AMP))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }
}
