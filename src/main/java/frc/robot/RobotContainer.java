package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.GenericCalibrateKF;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.RunWhenDisabledCommand;
import frc.robot.commands.TurnAndPositionToTargetCMD;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.commands.command_groups.BackupAuto;
import frc.robot.commands.command_groups.CollectCMDGP;
import frc.robot.commands.command_groups.ShootCMDGP;
import frc.robot.commands.command_groups.ShootWithPitcherCMDGP;
import frc.robot.commands.command_groups.ShootWithoutLimelight;
import frc.robot.constants.robots.RobotA;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.TrigonSwerveControllerCMDGP;
import frc.robot.subsystems.drivetrain.AutoWithPID;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.drivetrain.SupplierFieldDriveCMD;
import frc.robot.subsystems.drivetrain.ToggleMotorsModeCMD;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.intake_opener.IntakeOpenerCMD;
import frc.robot.subsystems.intake_opener.IntakeOpenerSS;
import frc.robot.subsystems.led.LedSS;
import frc.robot.subsystems.loader.LoaderCMD;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.pitcher.PitcherSS;
import frc.robot.subsystems.shooter.CalibrateShooterKfCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.subsystems.spinner.SpinnerSS;
import frc.robot.utilities.DashboardController;
import frc.robot.utilities.TrigonXboxController;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private final RobotA robotConstants;
    private final SubsystemContainerA subsystemContainer;
    private final TrigonXboxController driverXboxController;
    private final TrigonXboxController overrideXboxController;
    private final DashboardController dashboardController;
    private final PitcherLimelight limelight;

    private ShooterCMD shooterCMD;
    private CalibrateShooterKfCMD calibrateShooterKfCMD;
    private GenericCalibrateKF calibrateLoaderKfCMD;
    private SupplierFieldDriveCMD supplierFieldDriveCMD;
    private TurnToTargetCMD turnToTargetCMD;
    private TurnAndPositionToTargetCMD turnAndPositionToTargetCMD;
    private ToggleMotorsModeCMD toggleMotorsModeCMD;

    private TrigonSwerveControllerCMDGP motionTest;
    private IntakeOpenerCMD intakeCMD;

    private ShootCMDGP shootCMDGP;
    private ShootWithPitcherCMDGP ShootWithPitcherCMDGP;
    private CollectCMDGP collectCMDGP;

    private RunWhenDisabledCommand resetDirection;

    private AutoWithPID testAuto;

    /**
     * Add classes here
     */
    public RobotContainer() {

        Logger.configureLoggingAndConfig(this, true);
        robotConstants = new RobotA();
        subsystemContainer = new SubsystemContainerA();
        dashboardController = new DashboardController();
        driverXboxController = new TrigonXboxController(0);
        overrideXboxController = new TrigonXboxController(1);
        limelight = new PitcherLimelight(robotConstants.extendedLimelightConstants,
                robotConstants.retractedLimelightConstants, subsystemContainer.PITCHER_SS);

        initializeCommands();
        bindDriverCommands();
        bindOverrideCommands();
        setShuffleBoard();

        SmartDashboard.putData("Shooter Command", shooterCMD);
        SmartDashboard.putData("CalibrateShooterKfCMD", calibrateShooterKfCMD);
        SmartDashboard.putData("CalibrateLoaderKfCMD", calibrateLoaderKfCMD);
        SmartDashboard.putData("TurnToTargetCMD", turnToTargetCMD);
        SmartDashboard.putData("TurnAndPositionToTargetCMD", turnAndPositionToTargetCMD);
        SmartDashboard.putData("TrigonSwerveControllerCMDGP", motionTest);
        SmartDashboard.putNumber("Shooter/Desired Velocity", subsystemContainer.SHOOTER_SS.getVelocityRPM());

        Logger.configureLogging(subsystemContainer.DRIVETRAIN_SS);
        limelight.startVision(Target.PowerPort);
    }

    /**
     * Initializes all commands.
     */
    public void initializeCommands() {
        SmartDashboard.putNumber("Shooter/Desired Velocity", 0);
        shooterCMD = new ShooterCMD(subsystemContainer.SHOOTER_SS, null, robotConstants.shooterConstants,
                () -> SmartDashboard.getNumber("Shooter/Desired Velocity", 0));
        calibrateShooterKfCMD = new CalibrateShooterKfCMD(subsystemContainer.SHOOTER_SS,
                robotConstants.shooterConstants);
        supplierFieldDriveCMD = new SupplierFieldDriveCMD(subsystemContainer.DRIVETRAIN_SS,
                robotConstants.drivetrainConstants,
                () -> Math.signum(driverXboxController.getX(Hand.kRight)) * Math.pow(driverXboxController.getX(Hand.kRight), 2) / 1.5,
                () -> Math.signum(driverXboxController.getY(Hand.kRight)) * Math.pow(driverXboxController.getY(Hand.kRight), 2) / 1.5,
                () -> Math.signum(driverXboxController.getX(Hand.kLeft)) * Math.pow(driverXboxController.getX(Hand.kLeft), 2) / 1.5);

        motionTest = new TrigonSwerveControllerCMDGP(subsystemContainer.DRIVETRAIN_SS,
                robotConstants.motionProfilingConstants, AutoPath.Test);

        calibrateShooterKfCMD = new CalibrateShooterKfCMD(subsystemContainer.SHOOTER_SS,
                robotConstants.shooterConstants);
        calibrateLoaderKfCMD = new GenericCalibrateKF(subsystemContainer.LOADER_SS,
                robotConstants.loaderConstants.FEEDFORWARD_CONSTANTS);
\
        shootCMDGP = new ShootCMDGP(subsystemContainer, robotConstants, limelight);
        ShootWithPitcherCMDGP = new ShootWithPitcherCMDGP(subsystemContainer, robotConstants, limelight);
        collectCMDGP = new CollectCMDGP(subsystemContainer, robotConstants);
        intakeCMD = new IntakeOpenerCMD(true,subsystemContainer.INTAKE_OPENER_SS, robotConstants.intakeOpenerConstants);
        turnToTargetCMD = new TurnToTargetCMD(subsystemContainer.DRIVETRAIN_SS, limelight,
                robotConstants.visionConstants, Target.PowerPort);
        turnAndPositionToTargetCMD = new TurnAndPositionToTargetCMD(subsystemContainer.DRIVETRAIN_SS, limelight,
                robotConstants.visionConstants, Target.PowerPort);
        toggleMotorsModeCMD = new ToggleMotorsModeCMD(subsystemContainer.DRIVETRAIN_SS);
        resetDirection = new RunWhenDisabledCommand(() -> {
            subsystemContainer.DRIVETRAIN_SS.resetGyro();
            subsystemContainer.DRIVETRAIN_SS.resetOdometry(new Pose2d());
        });
        resetDirection.addRequirements(subsystemContainer.DRIVETRAIN_SS);

        testAuto = new AutoWithPID(robotConstants.drivetrainConstants.TRENCH_AUTO, subsystemContainer.DRIVETRAIN_SS, robotConstants.drivetrainConstants);
        SmartDashboard.putData("Drivetrain/Auto/AutoWithPIDCMD", testAuto);
    }

    /**
     * Binds all commands to the buttons that use them. Call this after initializing
     * the commands.
     */
    public void bindDriverCommands() {
        driverXboxController.getRightBumper().whenHeld(collectCMDGP).whenReleased(intakeCMD);
        driverXboxController.getButtonY().whenPressed(resetDirection);
        InstantCommand changeDriveRotation = new InstantCommand(() -> {
            subsystemContainer.DRIVETRAIN_SS.setAngle(subsystemContainer.DRIVETRAIN_SS.getAngle() + 90);
            subsystemContainer.DRIVETRAIN_SS.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(subsystemContainer.DRIVETRAIN_SS.getAngle())));
        });
        changeDriveRotation.addRequirements(subsystemContainer.DRIVETRAIN_SS);
        driverXboxController.getLeftBumper().whenPressed(changeDriveRotation);
        driverXboxController.getButtonX().whenPressed(
                new InstantCommand(subsystemContainer.PITCHER_SS::toggleSolenoid, subsystemContainer.PITCHER_SS));
        driverXboxController.getButtonB().whileHeld(
                new ShootCMDGP(subsystemContainer, robotConstants, limelight).withInterrupt(this::cancelShooterCMD));

        // robot.win=true
    }

    public void bindOverrideCommands() {
        overrideXboxController.getButtonA().whenPressed(subsystemContainer.INTAKE_OPENER_SS::toggleSolenoid);
        overrideXboxController.getLeftStickButton().whileHeld(new OverrideCommand(subsystemContainer.LOADER_SS, () -> overrideXboxController.getY(Hand.kLeft)));
        overrideXboxController.getRightStickButton().whileHeld(new OverrideCommand(subsystemContainer.SPINNER_SS, () -> overrideXboxController.getX(Hand.kRight)));
        overrideXboxController.getButtonX().whenHeld(new ShooterCMD(subsystemContainer.SHOOTER_SS, null, robotConstants.shooterConstants, () -> 3200));
        overrideXboxController.getButtonB().whileHeld(new ShootWithoutLimelight(subsystemContainer, robotConstants, () -> 3500)).whenReleased(new InstantCommand(() -> {subsystemContainer.PITCHER_SS.setSolenoidState(false);}));
        overrideXboxController.getButtonY().whileHeld(new ShootCMDGP(subsystemContainer, robotConstants,limelight,  3500));
    }

    public void setShuffleBoard() {
        SmartDashboard.putNumber("D-Pad", driverXboxController.getPOV());
        SmartDashboard.putData(" collect ", collectCMDGP);
        SmartDashboard.putData("ToggleMotorsModeCMD", toggleMotorsModeCMD);
        SmartDashboard.putData(" shoot ", ShootWithPitcherCMDGP);
        SmartDashboard.putData("toggle solenoid",
                new InstantCommand(subsystemContainer.PITCHER_SS::toggleSolenoid, subsystemContainer.PITCHER_SS));
        subsystemContainer.DRIVETRAIN_SS.setDefaultCommand(supplierFieldDriveCMD);
        SmartDashboard.putData("Loader/Load", new LoaderCMD(subsystemContainer.LOADER_SS,
                robotConstants.loaderConstants, robotConstants.loaderConstants.DEFAULT_SHOOTING_VELOCITY));
        SmartDashboard.putData("Shoot without pitcher CMDGP", shootCMDGP);
        subsystemContainer.DRIVETRAIN_SS.setDefaultCommand(supplierFieldDriveCMD);
        SmartDashboard.putData("Pitcher/Open", new InstantCommand(() -> subsystemContainer.PITCHER_SS.setSolenoidState(true)));
        SmartDashboard.putData("Pitcher/Close", new InstantCommand(() -> subsystemContainer.PITCHER_SS.setSolenoidState(false)));
        SmartDashboard.putBoolean("Pitcher/State", subsystemContainer.PITCHER_SS.getSolenoidState());
        SmartDashboard.putData("Drivetrain/ResetDirection", resetDirection);
    }

    /**
     * Checks the values of the driving joysticks and if one of them is above a
     * specified threshold. This is done incase the driver desires to continue //
     * moving before the robot is finished shooting.
     */
    private boolean cancelShooterCMD() {
        double threshold = robotConstants.shooterConstants.CANCEL_CMDGP_AXIS_THRESHOLD;
        return Math.abs(driverXboxController.getX(Hand.kRight)) >= threshold
                || Math.abs(driverXboxController.getY(Hand.kRight)) >= threshold
                || Math.abs(driverXboxController.getX(Hand.kLeft)) >= threshold;
    }

    public void updateDashboard() {
        dashboardController.update();
        Logger.updateEntries();
    }

    /**
     * Call this method in the autonomousInit.
     */
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(intakeCMD);
        CommandScheduler.getInstance().schedule(new BackupAuto(subsystemContainer, robotConstants, limelight));
    }

    /**
     * Call this method in the teleopInit.
     */
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(intakeCMD);
    }

    /**
     * Call this method periodically.
     */
    public void periodic() {
        updateDashboard();
        SmartDashboard.putNumber("Shooter/Velocity", subsystemContainer.SHOOTER_SS.getVelocityRPM());
        SmartDashboard.putNumber("Loader/Velocity", subsystemContainer.LOADER_SS.getVelocity());
        SmartDashboard.putNumber("PitcherLimelight/distance", limelight.calculateDistanceFromTower());   
    }

    public class SubsystemContainerA extends SubsystemContainer {
        public SubsystemContainerA() {
            // TODO: Set to subsystem when LEDs are added to the robot
            LED_SS = null;
            DRIVETRAIN_SS = new DrivetrainSS(robotConstants.drivetrainConstants);
            SHOOTER_SS = new ShooterSS(robotConstants.shooterConstants);
            PITCHER_SS = new PitcherSS(robotConstants.pitcherConstants);
            LOADER_SS = new LoaderSS(robotConstants.loaderConstants);
            SPINNER_SS = new SpinnerSS(robotConstants.spinnerConstants);
            INTAKE_SS = new IntakeSS(robotConstants.intakeConstants);
            INTAKE_OPENER_SS = new IntakeOpenerSS(robotConstants.intakeOpenerConstants);
        }
    }
}
