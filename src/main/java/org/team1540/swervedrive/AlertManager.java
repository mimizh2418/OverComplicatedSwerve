package org.team1540.swervedrive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.team1540.swervedrive.generated.TunerConstants;

/**
 * Class for managing high-level robot alerts, such as low battery and CAN bus errors.
 * Subsystem-specific alerts, such as motor disconnects, should be handled by their respective
 * subsystems.
 */
public class AlertManager {
    private static AlertManager instance;

    public static AlertManager getInstance() {
        if (instance == null) instance = new AlertManager();
        return instance;
    }

    private static final double lowBatteryDisableTime = 1.5;
    private static final double lowBatteryVoltageThreshold = 12.0;
    private static final double canErrorTimeThreshold = 0.5;

    private final Timer disabledTimer = new Timer();
    private final Timer canInitialErrorTimer = new Timer();
    private final Timer canErrorTimer = new Timer();
    private final Timer canivoreErrorTimer = new Timer();

    private int lastRioCanTEC = 0;
    private int lastRioCanREC = 0;
    private int lastCanivoreTEC = 0;
    private int lastCanivoreREC = 0;

    private final Alert canErrorAlert =
            new Alert(
                    "RIO CAN bus error, robot functionality may be severely limited",
                    Alert.AlertType.kError);
    private final Alert canivoreErrorAlert =
            new Alert(
                    "Swerve CAN bus error, drive functionality may be severely limited",
                    Alert.AlertType.kError);
    private final Alert lowBatteryAlert =
            new Alert("Battery voltage is low, consider replacing it", Alert.AlertType.kWarning);
    private final Alert tuningModeAlert =
            new Alert("Tuning mode is enabled", Alert.AlertType.kInfo);

    public void start() {
        disabledTimer.restart();
        canErrorTimer.restart();
        canivoreErrorTimer.restart();
        canInitialErrorTimer.restart();
        tuningModeAlert.set(Constants.isTuningMode());
    }

    public void update() {
        // Update timers
        CANStatus rioCanStatus = RobotController.getCANStatus();
        if (rioCanStatus.transmitErrorCount > lastRioCanTEC
                || rioCanStatus.receiveErrorCount > lastRioCanREC) {
            canErrorTimer.reset();
        }
        lastRioCanTEC = rioCanStatus.transmitErrorCount;
        lastRioCanREC = rioCanStatus.receiveErrorCount;

        CANBus.CANBusStatus canivoreStatus = TunerConstants.kCANBus.getStatus();
        if (!canivoreStatus.Status.isOK()
                || canivoreStatus.TEC > lastCanivoreTEC
                || canivoreStatus.REC > lastCanivoreREC) {
            canivoreErrorTimer.reset();
        }
        lastCanivoreTEC = canivoreStatus.TEC;
        lastCanivoreREC = canivoreStatus.REC;

        if (DriverStation.isEnabled()) disabledTimer.reset();

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
}
