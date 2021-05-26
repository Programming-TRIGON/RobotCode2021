package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.GenericCalibrateKF;
import frc.robot.commands.TurnAndPositionToTargetCMD;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.commands.command_groups.CollectCMDGP;
import frc.robot.commands.command_groups.ShootCMDGP;
import frc.robot.commands.command_groups.ShootWithPitcherCMDGP;
import frc.robot.constants.robots.RobotA;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.TrigonSwerveControllerCMDGP;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.drivetrain.SupplierFieldDriveCMD;
import frc.robot.subsystems.drivetrain.ToggleMotorsModeCMD;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.intake_opener.IntakeOpenerCMD;
import frc.robot.subsystems.intake_opener.IntakeOpenerSS;
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
    private final TrigonXboxController xboxController;
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
    private IntakeOpenerCMD closeIntakeCMD;

    private ShootCMDGP shootCMDGP;
    private ShootWithPitcherCMDGP ShootWithPitcherCMDGP;
    private CollectCMDGP collectCMDGP;

    /**
     * Add classes here
     */
    public RobotContainer() {

        Logger.configureLoggingAndConfig(this, true);
        robotConstants = new RobotA();
        subsystemContainer = new SubsystemContainerA();
        dashboardController = new DashboardController();
        xboxController = new TrigonXboxController(0);
        limelight = new PitcherLimelight(robotConstants.extendedLimelightConstants,
                robotConstants.retractedLimelightConstants, subsystemContainer.PITCHER_SS);

        initializeCommands();
        BindCommands();

        SmartDashboard.putData("Shooter Command", shooterCMD);
        SmartDashboard.putData("CalibrateShooterKfCMD", calibrateShooterKfCMD);
        SmartDashboard.putData("CalibrateLoaderKfCMD", calibrateLoaderKfCMD);
        SmartDashboard.putData("TurnToTargetCMD", turnToTargetCMD);
        SmartDashboard.putData("TurnAndPositionToTargetCMD", turnAndPositionToTargetCMD);
        SmartDashboard.putData("TrigonSwerveControllerCMDGP", motionTest);

        Logger.configureLogging(subsystemContainer.DRIVETRAIN_SS);
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
                () -> Math.signum(xboxController.getX(Hand.kRight)) * Math.pow(xboxController.getX(Hand.kRight), 2) / 6,
                () -> Math.signum(xboxController.getY(Hand.kRight)) * Math.pow(xboxController.getY(Hand.kRight), 2) / 6,
                () -> Math.signum(xboxController.getX(Hand.kLeft)) * Math.pow(xboxController.getX(Hand.kLeft), 2) / 4);

        motionTest = new TrigonSwerveControllerCMDGP(subsystemContainer.DRIVETRAIN_SS,
                robotConstants.motionProfilingConstants, AutoPath.Test);

        calibrateShooterKfCMD = new CalibrateShooterKfCMD(subsystemContainer.SHOOTER_SS,
                robotConstants.shooterConstants);
        calibrateLoaderKfCMD = new GenericCalibrateKF(subsystemContainer.LOADER_SS,
                robotConstants.loaderConstants.FEEDFORWARD_CONSTANTS);

        shootCMDGP = new ShootCMDGP(subsystemContainer, robotConstants, limelight);
        ShootWithPitcherCMDGP = new ShootWithPitcherCMDGP(subsystemContainer, robotConstants, limelight);
        collectCMDGP = new CollectCMDGP(subsystemContainer, robotConstants);
        closeIntakeCMD = new IntakeOpenerCMD(subsystemContainer.INTAKE_OPENER_SS, robotConstants.intakeOpenerConstants,
                () -> robotConstants.intakeOpenerConstants.DEFAULT_CLOSE_POWER);
        turnToTargetCMD = new TurnToTargetCMD(subsystemContainer.DRIVETRAIN_SS, limelight,
                robotConstants.visionConstants, Target.PowerPort);
        turnAndPositionToTargetCMD = new TurnAndPositionToTargetCMD(subsystemContainer.DRIVETRAIN_SS, limelight,
                robotConstants.visionConstants, Target.PowerPort);
        toggleMotorsModeCMD = new ToggleMotorsModeCMD(subsystemContainer.DRIVETRAIN_SS);
        robotConstants.pcm.compressorMap.COMPRESSOR.stop();
    }

    /**
     * Binds all commands to the buttons that use them. Call this after initializing
     * the commands.
     */
    public void BindCommands() {
        xboxController.getButtonA().whenHeld(collectCMDGP).whenReleased(closeIntakeCMD);
        xboxController.getButtonY().whenPressed(new InstantCommand(() -> {
            subsystemContainer.DRIVETRAIN_SS.resetGyro();
            subsystemContainer.DRIVETRAIN_SS.resetOdometry(new Pose2d());
        }));
        xboxController.getButtonX().toggleWhenPressed(
                new InstantCommand(subsystemContainer.PITCHER_SS::toggleSolenoid, subsystemContainer.PITCHER_SS));
        xboxController.getButtonB().whenHeld(
                new ShootCMDGP(subsystemContainer, robotConstants, limelight).withInterrupt(this::cancelShooterCMD));
        SmartDashboard.putNumber("D-Pad", xboxController.getPOV());

        SmartDashboard.putData(" collect ", collectCMDGP);
        SmartDashboard.putData("ToggleMotorsModeCMD", toggleMotorsModeCMD);

        SmartDashboard.putData(" shoot ", ShootWithPitcherCMDGP);
        SmartDashboard.putData("toggle solenoid",
                new InstantCommand(subsystemContainer.PITCHER_SS::toggleSolenoid, subsystemContainer.PITCHER_SS));
        subsystemContainer.DRIVETRAIN_SS.setDefaultCommand(supplierFieldDriveCMD);
        SmartDashboard.putData("Loader/Load", new LoaderCMD(subsystemContainer.LOADER_SS,
                robotConstants.loaderConstants, robotConstants.loaderConstants.DEFAULT_SHOOTING_VELOCITY));
        SmartDashboard.putData("Shoot without pitcher CMDGP", shootCMDGP);
        xboxController.getLeftBumper().whenPressed(new InstantCommand(() -> {
            subsystemContainer.SHOOTER_SS.areaCounter++;
            SmartDashboard.putNumber("Shooter/Desired Velocity",
                    robotConstants.shooterConstants.AREA_ARRAY[subsystemContainer.SHOOTER_SS.areaCounter]);
        }));
        SmartDashboard.putNumber("Shooter/Desired Velocity",
                robotConstants.shooterConstants.AREA_ARRAY[subsystemContainer.SHOOTER_SS.areaCounter]);
        subsystemContainer.DRIVETRAIN_SS.setDefaultCommand(supplierFieldDriveCMD);
    }

    /**
     * Checks the values of the driving joysticks and if one of them is above a
     * specified threshold. This is done incase the driver desires to continue //
     * moving before the robot is finished shooting.
     */
    private boolean cancelShooterCMD() {
        double threshold = robotConstants.shooterConstants.CANCEL_CMDGP_AXIS_THRESHOLD;
        return Math.abs(xboxController.getX(Hand.kRight)) >= threshold
                || Math.abs(xboxController.getY(Hand.kRight)) >= threshold
                || Math.abs(xboxController.getX(Hand.kLeft)) >= threshold;
    }

    public void updateDashboard() {
        dashboardController.update();
        Logger.updateEntries();
    }

    /**
     * Call this method in the autonomousInit.
     */
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(closeIntakeCMD);
    }

    /**
     * Call this method in the teleopInit.
     */
    public void teleopInit() {
        CommandScheduler.getInstance().schedule(closeIntakeCMD);
    }

    /**
     * Call this method periodically.
     */
    public void periodic() {
        updateDashboard();
        SmartDashboard.putNumber("Shooter/Velocity", subsystemContainer.SHOOTER_SS.getVelocityRPM());
        SmartDashboard.putNumber("Loader/Velocity", subsystemContainer.LOADER_SS.getVelocity());
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
