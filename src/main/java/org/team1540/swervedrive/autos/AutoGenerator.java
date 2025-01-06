package org.team1540.swervedrive.autos;

import choreo.auto.AutoFactory;
import org.team1540.swervedrive.RobotState;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;

public class AutoGenerator {
    private final RobotState robotState = RobotState.getInstance();

    private final AutoFactory autoFactory;

    private final Drivetrain drivetrain;

    public AutoGenerator(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        autoFactory = new AutoFactory(
                robotState::getRobotPose,
                robotState::resetPose,
                drivetrain::followTrajectory,
                true,
                drivetrain,
                (trajectory, starting) -> {
                    if (starting) robotState.setActiveTrajectory(trajectory);
                    else robotState.clearActiveTrajectory();
                });
    }
}
