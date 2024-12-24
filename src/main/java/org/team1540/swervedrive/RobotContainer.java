package org.team1540.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team1540.swervedrive.commands.ScoringCommands;
import org.team1540.swervedrive.subsystems.arm.Arm;
import org.team1540.swervedrive.subsystems.drive.*;
import org.team1540.swervedrive.subsystems.vision.AprilTagVision;
import org.team1540.swervedrive.util.AllianceFlipUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();

    // Subsystems
    public final Drivetrain drivetrain;
    public final Arm arm;
    public final AprilTagVision aprilTagVision;

    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = Drivetrain.createReal();
                arm = Arm.createReal();
                aprilTagVision = AprilTagVision.createReal();
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();
                arm = Arm.createSim();
                aprilTagVision = AprilTagVision.createSim();

                RobotState.getInstance().resetPose(FieldConstants.MIDFIELD);
                break;

            default:
                // Replayed robot, disable IO implementations
                drivetrain = Drivetrain.createDummy();
                arm = Arm.createDummy();
                aprilTagVision = AprilTagVision.createDummy();
                break;
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        if (Constants.isTuningMode()) {
            autoChooser.addOption("Drive FF Characterization", drivetrain.feedforwardCharacterization());
            autoChooser.addOption("Drive Wheel Radius Characterization", drivetrain.wheelRadiusCharacterization());
            autoChooser.addOption("Arm FF Characterization", arm.feedforwardCharacterization());
        }

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.povUp()
                .toggleOnTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.reverseRotation(Rotation2d.kZero), () -> true));
        driver.povLeft()
                .toggleOnTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.reverseRotation(Rotation2d.kCCW_90deg), () -> true));
        driver.povDown()
                .toggleOnTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.reverseRotation(Rotation2d.k180deg), () -> true));
        driver.povRight()
                .toggleOnTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.reverseRotation(Rotation2d.kCW_90deg), () -> true));

        Command speakerAimCommand =
                ScoringCommands.teleopSpeakerAimCommand(driver.getHID(), drivetrain, arm, () -> true);
        Command ampStageCommand = ScoringCommands.teleopAmpStageCommand(driver.getHID(), drivetrain, arm, () -> true);
        driver.rightBumper().toggleOnTrue(speakerAimCommand);
        driver.y().toggleOnTrue(ampStageCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
