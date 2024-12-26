package org.team1540.swervedrive.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.SimulatedArena;
import org.team1540.swervedrive.Constants;
import org.team1540.swervedrive.FieldConstants;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.commands.SuperstructureCommands;
import org.team1540.swervedrive.subsystems.arm.Arm;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.subsystems.indexer.Indexer;
import org.team1540.swervedrive.subsystems.shooter.Shooter;

public class AutoGenerator {
    private final RobotState robotState = RobotState.getInstance();

    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;
    private final Indexer indexer;
    private final Arm arm;
    private final Shooter shooter;

    public AutoGenerator(Drivetrain drivetrain, Indexer indexer, Arm arm, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.arm = arm;
        this.shooter = shooter;

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

    public AutoRoutine centerLanePCBA() {
        AutoRoutine routine = autoFactory.newRoutine("Center Lane PCBA");
        AutoTrajectory traj = routine.trajectory("CenterLanePCBA");
        resetPoseInSim(routine, traj);

        routine.active().onTrue(traj.cmd());

        routine.active().whileTrue(SuperstructureCommands.speakerAimCommand(arm, shooter));
        traj.atTranslation("StartShoot").onTrue(indexer.persistentStateCommand(Indexer.IndexerState.FEED_SHOOTER));

        return routine;
    }

    public AutoRoutine centerLanePCBADEFSprint() {
        AutoRoutine routine = autoFactory.newRoutine("Center Lane PCBADEF Sprint");
        AutoTrajectory traj = routine.trajectory("CenterLanePCBADEFSprint");
        resetPoseInSim(routine, traj);

        routine.active().whileTrue(traj.cmd());

        routine.active()
                .and(() -> FieldConstants.inOwnWing(RobotState.getInstance().getRobotPose()))
                .whileTrue(SuperstructureCommands.speakerAimCommand(arm, shooter));
        traj.atTranslation("StartContinuousShoot")
                .onTrue(indexer.persistentStateCommand(Indexer.IndexerState.FEED_SHOOTER));
        traj.atTime("StopContinuousShoot").onTrue(indexer.continuousIntakeCommand());
        traj.atTranslation("ShootD").onTrue(indexer.feedShooterCommand().andThen(indexer.continuousIntakeCommand()));
        traj.atTranslation("ShootE").onTrue(indexer.feedShooterCommand().andThen(indexer.continuousIntakeCommand()));
        traj.atTranslation("ShootF").onTrue(indexer.feedShooterCommand().andThen(indexer.continuousIntakeCommand()));

        return routine;
    }
}
