package org.team1540.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team1540.swervedrive.subsystems.drive.*;

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

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = Drivetrain.createReal();
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();

                RobotState.getInstance().resetPose(new Pose2d(3.0, 3.0, new Rotation2d()));
                break;

            default:
                // Replayed robot, disable IO implementations
                drivetrain = Drivetrain.createDummy();
                break;
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        if (Constants.isTuningMode()) {
            autoChooser.addOption("Drive FF Characterization", drivetrain.feedforwardCharacterization());
            autoChooser.addOption("Drive Wheel Radius Characterization", drivetrain.wheelRadiusCharacterization());
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
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(
                () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX(), () -> true));
        controller.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        controller.y().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        controller.start().and(Robot::isSimulation).onTrue(Commands.runOnce(() -> RobotState.getInstance()
                .resetPose(new Pose2d(3.0, 3.0, new Rotation2d()))));
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
