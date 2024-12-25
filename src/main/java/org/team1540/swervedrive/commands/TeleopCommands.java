package org.team1540.swervedrive.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.subsystems.arm.Arm;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.subsystems.shooter.Shooter;

public class TeleopCommands {
    private static final double LOW_PASSING_DISTANCE_THRESHOLD = 7.5;

    public static Command teleopSpeakerAimCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, Shooter shooter, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                drivetrain.teleopDriveWithHeadingCommand(
                        driver,
                        () -> RobotState.getInstance()
                                .getSpeakerAimingParameters()
                                .driveHeading(),
                        isFieldRelative),
                SuperstructureCommands.speakerAimCommand(arm, shooter));
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
                SuperstructureCommands.passingAimCommand(arm, shooter));
    }

    public static Command teleopLowPassingAimCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, Shooter shooter, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                drivetrain.teleopDriveWithHeadingCommand(
                        driver,
                        () -> RobotState.getInstance()
                                .getPassingAimingParameters()
                                .driveHeading(),
                        isFieldRelative),
                SuperstructureCommands.lowPassingAimCommand(arm, shooter));
    }

    public static Command teleopDynamicAimCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, Shooter shooter, BooleanSupplier isFieldRelative) {
        var robotState = RobotState.getInstance();
        return Commands.deadline(
                        drivetrain.teleopDriveWithHeadingCommand(
                                driver,
                                () -> FieldConstants.inOwnWing(robotState.getRobotPose())
                                        ? robotState
                                                .getSpeakerAimingParameters()
                                                .driveHeading()
                                        : robotState
                                                .getPassingAimingParameters()
                                                .driveHeading(),
                                isFieldRelative),
                        Commands.repeatingSequence(Commands.defer(
                                () -> {
                                    if (FieldConstants.inOwnWing(robotState.getRobotPose()))
                                        return arm.requestStateCommand(Arm.ArmState.SPEAKER);
                                    else if (robotState
                                                    .getPassingAimingParameters()
                                                    .effectiveDistanceMeters()
                                            <= LOW_PASSING_DISTANCE_THRESHOLD)
                                        return arm.requestStateCommand(Arm.ArmState.LOW_PASS);
                                    else return arm.requestStateCommand(Arm.ArmState.PASS);
                                },
                                Set.of(arm))),
                        Commands.repeatingSequence(Commands.defer(
                                () -> {
                                    if (FieldConstants.inOwnWing(robotState.getRobotPose()))
                                        return shooter.requestStateCommand(Shooter.ShooterState.SPEAKER);
                                    else if (robotState
                                                    .getPassingAimingParameters()
                                                    .effectiveDistanceMeters()
                                            <= LOW_PASSING_DISTANCE_THRESHOLD)
                                        return shooter.requestStateCommand(Shooter.ShooterState.LOW_PASS);
                                    else return shooter.requestStateCommand(Shooter.ShooterState.PASS);
                                },
                                Set.of(shooter))))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)))
                .finallyDo(shooter::stop);
    }

    public static Command teleopStageAmpCommand(
            XboxController driver, Drivetrain drivetrain, Arm arm, BooleanSupplier isFieldRelative) {
        return Commands.parallel(
                        drivetrain.teleopDriveWithHeadingCommand(driver, () -> Rotation2d.kCW_90deg, isFieldRelative),
                        arm.requestStateCommand(Arm.ArmState.AMP))
                .finallyDo(() -> CommandScheduler.getInstance().schedule(arm.requestStateCommand(Arm.ArmState.STOW)));
    }
}
