package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.command_groups.ShootCMDGP;
import frc.robot.commands.command_groups.CollectCMDGP;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.intake_opener.IntakeOpenerSS;
import frc.robot.subsystems.intake_opener.IntakeOpenerCMD;
import frc.robot.subsystems.led.LedSS;
import frc.robot.subsystems.loader.LoaderSS;
import frc.robot.subsystems.pitcher.PitcherSS;
import frc.robot.subsystems.shooter.CalibrateShooterKfCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.subsystems.spinner.SpinnerSS;
import frc.robot.utilities.DashboardController;
import frc.robot.utilities.TrigonXboxController;
import frc.robot.vision.limelights.PitcherLimelight;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private final RobotA robotConstants;
    private final HomeField fieldConstants;
    private final SubsystemContainerA subsystemContainer;
    private final TrigonXboxController xboxController;
    private final DashboardController dashboardController;
    private final PitcherLimelight limelight;

    private ShooterCMD shooterCMD;
    private CalibrateShooterKfCMD calibrateShooterKfCMD;
    private IntakeOpenerCMD closeIntakeCMD;

    private Command shootCMDGP;
    private CollectCMDGP collectCMDGP;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        subsystemContainer = new SubsystemContainerA();
        dashboardController = new DashboardController();
        xboxController = new TrigonXboxController(0);
        limelight = new PitcherLimelight(robotConstants.extendedLimelightConstants,
                robotConstants.retractedLimelightConstants, subsystemContainer.PITCHER_SS);

        initializeCommands();
        BindCommands();

        SmartDashboard.putData("Shooter Command", shooterCMD);
        SmartDashboard.putData("CalibrateShooterKfCMD", calibrateShooterKfCMD);
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

        shootCMDGP = new ShootCMDGP(subsystemContainer, robotConstants, limelight)
                .withInterrupt(this::cancelShooterCMDGP);
        collectCMDGP = new CollectCMDGP(subsystemContainer, robotConstants);
        closeIntakeCMD = new IntakeOpenerCMD(subsystemContainer.INTAKE_OPENER_SS, robotConstants.intakeOpenerConstants,
                () -> robotConstants.intakeOpenerConstants.DEFAULT_CLOSE_POWER);
    }

    /**
     * Binds all commands to the buttons that use them. Call this after initializing
     * the commands.
     */
    public void BindCommands() {
        xboxController.getButtonX().whenPressed(shootCMDGP);
        xboxController.getButtonA().whenHeld(collectCMDGP).whenReleased(closeIntakeCMD);
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
    }

    /**
     * Checks the values of the driving joysticks and if one of them is above a
     * specified threshold. This is done incase the driver desires to continue
     * moving before the robot is finished shooting.
     * 
     * @return if to interrupt the shooterCMDGP.
     */
    private boolean cancelShooterCMDGP() {
        double threshold = robotConstants.shooterConstants.CANCEL_CMDGP_AXIS_THRESHOLD;
        return Math.abs(xboxController.getX(Hand.kRight)) >= threshold
                || Math.abs(xboxController.getY(Hand.kRight)) >= threshold
                || Math.abs(xboxController.getX(Hand.kLeft)) >= threshold;
    }

    public class SubsystemContainerA extends SubsytemContainer {
        public SubsystemContainerA() {
            LED_SS = new LedSS(robotConstants.ledConstants);
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
