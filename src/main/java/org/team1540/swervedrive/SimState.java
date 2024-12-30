package org.team1540.swervedrive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.swervedrive.subsystems.intake.Intake;
import org.team1540.swervedrive.subsystems.pivot.Pivot;
import org.team1540.swervedrive.subsystems.turret.Turret;
import org.team1540.swervedrive.util.AllianceFlipUtil;

public class SimState {
    private static SimState instance = null;

    public static SimState getInstance() {
        if (instance == null) {
            instance = new SimState();
        }
        return instance;
    }

    private static final double INTAKE_TRANSFER_TIME_SECS = 0.15;
    private static final double FEEDER_TRANSFER_TIME_SECS = 0.15;

    private static final double INTAKE_VOLTAGE_INTEGRAL_THRESH = 12.0 * INTAKE_TRANSFER_TIME_SECS;
    private static final double FEEDER_VOLTAGE_INTEGRAL_THRESH = 12.0 * FEEDER_TRANSFER_TIME_SECS;

    private static final double NOTE_VELOCITY_COEFF_MPS_PER_RPM = 14.25 / 8500;

    private boolean configured = false;
    private SwerveDriveSimulation driveSim;
    private IntakeSimulation intakeSim;

    private double avgShooterRPM = 0.0;

    @AutoLogOutput(key = "SimState/IntakeRunning")
    private boolean intakeRunning = false;

    private boolean intakeHasNote = false;
    private double intakeTopVoltage = 0.0;
    private double intakeBottomVoltage = 0.0;

    boolean feederHasNote = false;
    private double teacupVoltage = 0.0;
    private double feederVoltage = 0.0;

    private double intakeVoltageIntegral = 0.0;
    private double feederVoltageIntegral = 0.0;

    public void configure(SwerveDriveSimulation driveSim) {
        if (!configured) {
            this.driveSim = driveSim;
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

            intakeSim = new IntakeSimulation(
                    "Note",
                    driveSim,
                    Meters.of(Intake.WIDTH_METERS),
                    Meters.of(Intake.EXTENSION_METERS + 0.01),
                    IntakeSimulation.IntakeSide.FRONT,
                    1);
            intakeSim.register();
            configured = true;
        }
    }

    public Pose2d getSimulatedRobotPose() {
        if (!configured) return Pose2d.kZero;
        return driveSim.getSimulatedDriveTrainPose();
    }

    public void resetSimPose(Pose2d pose) {
        if (!configured) return;
        driveSim.setSimulationWorldPose(pose);
    }

    public void addShooterVelocityData(double leftRPM, double rightRPM) {
        if (!configured) return;
        avgShooterRPM = (leftRPM + rightRPM) / 2.0;
    }

    public void addIntakeData(double topVoltage, double bottomVoltage) {
        if (!configured) return;
        this.intakeTopVoltage = topVoltage;
        this.intakeBottomVoltage = bottomVoltage;
    }

    public void addFeederData(double teacupVoltage, double feederVoltage) {
        if (!configured) return;
        this.teacupVoltage = teacupVoltage;
        this.feederVoltage = feederVoltage;
    }

    @AutoLogOutput(key = "SimState/IntakeHasNote")
    public boolean noteInIntake() {
        return intakeHasNote;
    }

    @AutoLogOutput(key = "SimState/FeederHasNote")
    public boolean noteInFeeder() {
        return feederHasNote;
    }

    public void update() {
        if (Constants.currentMode != Constants.Mode.SIM || !configured) return;
        Logger.recordOutput("SimState/SimulatedRobotPose", getSimulatedRobotPose());
        Logger.recordOutput(
                "SimState/Notes",
                SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));

        intakeHasNote = intakeSim.getGamePiecesAmount() > 0 && !feederHasNote;

        intakeRunning = Math.abs(intakeTopVoltage) >= 4.0;

        if (intakeRunning) {
            if (intakeTopVoltage + intakeBottomVoltage > 0.0) intakeSim.startIntake();
            if (intakeHasNote) {
                intakeVoltageIntegral += Constants.LOOP_PERIOD_SECS * (intakeTopVoltage + intakeBottomVoltage) / 2.0;
                if (intakeVoltageIntegral > INTAKE_VOLTAGE_INTEGRAL_THRESH) {
                    intakeHasNote = false;
                    feederHasNote = true;
                }
                if (intakeVoltageIntegral < -INTAKE_VOLTAGE_INTEGRAL_THRESH) {
                    intakeHasNote = false;
                    ejectNote();
                }
            } else intakeVoltageIntegral = 0.0;
        } else intakeSim.stopIntake();

        if (feederHasNote) {
            feederVoltageIntegral += Constants.LOOP_PERIOD_SECS * (teacupVoltage + feederVoltage) / 2.0;
            if (feederVoltageIntegral > FEEDER_VOLTAGE_INTEGRAL_THRESH) {
                feederHasNote = false;
                shootNote();
            }
            if (feederVoltageIntegral < -FEEDER_VOLTAGE_INTEGRAL_THRESH) {
                feederHasNote = false;
                intakeHasNote = true;
            }
        } else feederVoltageIntegral = 0.0;

        SimulatedArena.getInstance().simulationPeriodic();
    }

    private void ejectNote() {
        if (!configured) return;
        if (!intakeSim.obtainGamePieceFromIntake()) return;
        SimulatedArena.getInstance()
                .addGamePiece(new CrescendoNoteOnField(getSimulatedRobotPose()
                        .transformBy(
                                new Transform2d((Constants.BUMPER_LENGTH_X_METERS / 2) + 0.05, 0.0, Rotation2d.kZero))
                        .getTranslation()));
    }

    private void shootNote() {
        if (!configured) return;
        if (!intakeSim.obtainGamePieceFromIntake()) return;
        Pose3d robotToShooter = new Pose3d(
                        Turret.ORIGIN_METERS,
                        new Rotation3d(RobotState.getInstance().getTurretAngle()))
                .transformBy(new Transform3d(
                        Pivot.TURRET_TO_PIVOT_METERS,
                        new Rotation3d(
                                0.0, -RobotState.getInstance().getPivotAngle().getRadians(), 0.0)))
                .transformBy(new Transform3d(Pivot.LENGTH_METERS, 0.0, Units.inchesToMeters(1.25), Rotation3d.kZero));
        ChassisSpeeds turretSpeeds = driveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        turretSpeeds.omegaRadiansPerSecond +=
                Units.rotationsToRadians(RobotState.getInstance().getTurretVelocityRPS());
        GamePieceProjectile note = new NoteOnFly(
                        getSimulatedRobotPose().getTranslation(),
                        robotToShooter
                                .getTranslation()
                                .toTranslation2d()
                                .rotateBy(AllianceFlipUtil.maybeReverseRotation(
                                        getSimulatedRobotPose().getRotation().plus(Rotation2d.k180deg))),
                        turretSpeeds.div(2.0),
                        getSimulatedRobotPose()
                                .getRotation()
                                .plus(RobotState.getInstance().getTurretAngle()),
                        robotToShooter.getZ(),
                        avgShooterRPM * NOTE_VELOCITY_COEFF_MPS_PER_RPM,
                        RobotState.getInstance().getPivotAngle().getRadians())
                .enableBecomeNoteOnFieldAfterTouchGround()
                .withTouchGroundHeight(0.05);
        SimulatedArena.getInstance().addGamePieceProjectile(note);
    }
}
