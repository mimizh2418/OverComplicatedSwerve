package org.team1540.swervedrive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team1540.swervedrive.subsystems.drive.Drivetrain;
import org.team1540.swervedrive.util.Alert;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private static final double lowBatteryDisableTime = 1.5;
    private static final double lowBatteryVoltageThreshold = 12.0;
    private static final double canErrorTimeThreshold = 0.5;

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    private final Timer disabledTimer = new Timer();
    private final Timer canInitialErrorTimer = new Timer();
    private final Timer canErrorTimer = new Timer();
    private final Timer canivoreErrorTimer = new Timer();

    private final CANStatus lastCanStatus = new CANStatus();
    private final CANBus.CANBusStatus lastCanivoreStatus = new CANBus.CANBusStatus();

    private final Alert canErrorAlert =
            new Alert("RIO CAN bus error, robot functionality may be severely limited", Alert.AlertType.ERROR);
    private final Alert canivoreErrorAlert =
            new Alert("Swerve CAN bus error, drive functionality may be severely limited", Alert.AlertType.ERROR);
    private final Alert lowBatteryAlert =
            new Alert("Battery voltage is low, consider replacing it", Alert.AlertType.WARNING);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();

        disabledTimer.restart();
        canErrorTimer.restart();
        canivoreErrorTimer.restart();
        canInitialErrorTimer.restart();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /**
     * This function is called periodically during all modes.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Update CAN alert timers
        CANStatus canStatus = RobotController.getCANStatus();
        if (canStatus.transmitErrorCount > lastCanStatus.transmitErrorCount
                || canStatus.receiveErrorCount > lastCanStatus.receiveErrorCount){
            canErrorTimer.reset();
        }
        lastCanStatus.setStatus(
                canStatus.percentBusUtilization,
                canStatus.busOffCount,
                canStatus.txFullCount,
                canStatus.receiveErrorCount,
                canStatus.transmitErrorCount);

        CANBus.CANBusStatus canivoreStatus = CANBus.getStatus(Drivetrain.CAN_BUS);
        if (!canivoreStatus.Status.isOK()
                || canivoreStatus.REC > lastCanivoreStatus.REC
                || canivoreStatus.TEC > lastCanivoreStatus.TEC) {
            canivoreErrorTimer.reset();
        }
        lastCanivoreStatus.Status = canivoreStatus.Status;
        lastCanivoreStatus.BusUtilization = canivoreStatus.BusUtilization;
        lastCanivoreStatus.BusOffCount = canivoreStatus.BusOffCount;
        lastCanivoreStatus.TxFullCount = canivoreStatus.TxFullCount;
        lastCanivoreStatus.REC = canivoreStatus.REC;
        lastCanivoreStatus.TEC = canivoreStatus.TEC;

        if (DriverStation.isEnabled()) disabledTimer.reset();

        // Update alerts
        canErrorAlert.set(
                !canErrorTimer.hasElapsed(canErrorTimeThreshold)
                && canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
        canivoreErrorAlert.set(
                !canivoreErrorTimer.hasElapsed(canErrorTimeThreshold)
                && canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
        lowBatteryAlert.set(
                RobotController.getBatteryVoltage() < lowBatteryVoltageThreshold
                        && disabledTimer.hasElapsed(lowBatteryDisableTime));
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.drivetrain.zeroFieldOrientation();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
