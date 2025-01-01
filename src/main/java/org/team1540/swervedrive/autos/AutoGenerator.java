package org.team1540.swervedrive.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.AimingCommands;
import org.team1540.swervedrive.commands.IntakeCommands;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.subsystems.feeder.Feeder;
import org.team1540.swervedrive.subsystems.intake.Intake;
import org.team1540.swervedrive.subsystems.pivot.Pivot;
import org.team1540.swervedrive.subsystems.shooter.Shooter;
import org.team1540.swervedrive.subsystems.turret.Turret;

public class AutoGenerator {
    private final RobotState robotState = RobotState.getInstance();

    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Turret turret;
    private final Pivot pivot;

    public AutoGenerator(
            Drivetrain drivetrain, Intake intake, Feeder feeder, Shooter shooter, Turret turret, Pivot pivot) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.turret = turret;
        this.pivot = pivot;

        autoFactory = new AutoFactory(
                robotState::getRobotPose,
                robotState::resetPose,
                drivetrain::followTrajectory,
                true,
                drivetrain,
                new AutoFactory.AutoBindings(),
                (trajectory, starting) -> {
                    if (starting) robotState.setActiveTrajectory(trajectory);
                    else robotState.clearActiveTrajectory();
                });
    }

    private void resetPoseInSim(AutoRoutine routine, AutoTrajectory startingTrajectory) {
        if (Constants.currentMode == Constants.Mode.SIM) {
            routine.active().onTrue(Commands.runOnce(() -> {
                robotState.resetPose(startingTrajectory.getInitialPose().orElse(FieldConstants.MIDFIELD));
                SimulatedArena.getInstance().resetFieldForAuto();
            }));
        }
    }

    public AutoRoutine sourceLanePCBADEFSprint() {
        AutoRoutine routine = autoFactory.newRoutine("Source Lane PCBADEF Sprint");
        AutoTrajectory traj = routine.trajectory("SourceLanePCBADEFSprint");
        resetPoseInSim(routine, traj);

        routine.active().whileTrue(traj.cmd());
        routine.active()
                .onTrue(Commands.runOnce(
                        () -> intake.setDefaultCommand(IntakeCommands.continuousIntakeCommand(intake, feeder))));
        routine.anyDone(traj).onTrue(Commands.runOnce(intake::removeDefaultCommand));
        routine.active().whileTrue(AimingCommands.speakerAimCommand(turret, pivot, shooter));

        traj.atTranslation("StartContinuousShoot")
                .onTrue(IntakeCommands.continuousFeedCommand(intake, feeder)
                        .until(traj.atTranslation("StopContinuousShoot")));
        traj.atTranslation("ShootD").onTrue(IntakeCommands.feedCommand(intake, feeder));
        traj.atTranslation("ShootE").onTrue(IntakeCommands.feedCommand(intake, feeder));
        traj.atTranslation("ShootF").onTrue(IntakeCommands.feedCommand(intake, feeder));

        return routine;
    }

    public AutoRoutine ampLanePDEFABC() {
        AutoRoutine routine = autoFactory.newRoutine("Amp Lane PDEFABC");
        AutoTrajectory traj = routine.trajectory("AmpLanePDEFABC");
        resetPoseInSim(routine, traj);

        routine.active().whileTrue(traj.cmd());
        routine.active()
                .onTrue(Commands.runOnce(
                        () -> intake.setDefaultCommand(IntakeCommands.continuousIntakeCommand(intake, feeder))));
        routine.anyDone(traj).onTrue(Commands.runOnce(intake::removeDefaultCommand));
        routine.active().whileTrue(AimingCommands.speakerAimCommand(turret, pivot, shooter));

        traj.atTranslation("ShootP").onTrue(IntakeCommands.feedCommand(intake, feeder));
        traj.atTranslation("ShootD").onTrue(IntakeCommands.feedCommand(intake, feeder));
        traj.atTranslation("ShootE").onTrue(IntakeCommands.feedCommand(intake, feeder));
        traj.atTranslation("ShootF").onTrue(IntakeCommands.feedCommand(intake, feeder));
        traj.atTranslation("StartContinuousShoot", 0.25).onTrue(IntakeCommands.continuousFeedCommand(intake, feeder));

        return routine;
    }
}
