package org.team1540.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.swervedrive.autos.AutoGenerator;
import org.team1540.swervedrive.subsystems.drive.*;
import org.team1540.swervedrive.subsystems.vision.AprilTagVision;
import org.team1540.swervedrive.util.AllianceFlipUtil;
import org.team1540.swervedrive.util.auto.LoggedAutoChooser;

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
    public final AprilTagVision aprilTagVision;

    // Controller
    private final CommandXboxController driver = new CommandXboxController(0);

    // Auto generator
    private final AutoGenerator autoGenerator;

    // Dashboard inputs
    private final LoggedAutoChooser autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain = Drivetrain.createReal();
                aprilTagVision = AprilTagVision.createReal();
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain = Drivetrain.createSim();
                aprilTagVision = AprilTagVision.createSim();

                SimulatedArena.getInstance().resetFieldForAuto();
                break;

            default:
                // Replayed robot, disable IO implementations
                drivetrain = Drivetrain.createDummy();
                aprilTagVision = AprilTagVision.createDummy();
                break;
        }

        autoGenerator = new AutoGenerator(drivetrain);
        autoChooser = new LoggedAutoChooser("Auto Chooser");

        if (Constants.isTuningMode()) {
            autoChooser.addCmd("Drive FF Characterization", drivetrain::feedforwardCharacterization);
            autoChooser.addCmd("Drive Wheel Radius Characterization", drivetrain::wheelRadiusCharacterization);
        }

        // Configure periodic callbacks
        configurePeriodicCallbacks();

        // Configure robot mode triggers
        configureRobotModeTriggers();

        // Configure the button bindings
        configureButtonBindings();

        // Configure auto routines
        configureAutoRoutines();
    }

    private void configurePeriodicCallbacks() {
        CommandScheduler.getInstance()
                .schedule(Commands.run(AlertManager.getInstance()::update)
                        .ignoringDisable(true)
                        .withName("Periodic Callbacks"));

        if (Constants.currentMode == Constants.Mode.SIM) {
            CommandScheduler.getInstance()
                    .schedule(Commands.run(robotState::updateSimState)
                            .ignoringDisable(true)
                            .withName("Simulation update"));
        }
    }

    private void configureRobotModeTriggers() {
        RobotModeTriggers.teleop()
                .and(DriverStation::isFMSAttached)
                .onTrue(Commands.runOnce(drivetrain::zeroFieldOrientation));
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
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kZero), () -> true));
        driver.povLeft()
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(),
                        () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCCW_90deg),
                        () -> true));
        driver.povDown()
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(), () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.k180deg), () -> true));
        driver.povRight()
                .onTrue(drivetrain.teleopDriveWithHeadingCommand(
                        driver.getHID(),
                        () -> AllianceFlipUtil.maybeReverseRotation(Rotation2d.kCW_90deg),
                        () -> true));

        if (Constants.currentMode == Constants.Mode.SIM) {
            driver.back()
                    .onTrue(Commands.runOnce(() -> {
                                SimulatedArena.getInstance().resetFieldForAuto();
                                robotState.resetPose(FieldConstants.MIDFIELD);
                            })
                            .ignoringDisable(true));
        }
    }

    private void configureAutoRoutines() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }
}
