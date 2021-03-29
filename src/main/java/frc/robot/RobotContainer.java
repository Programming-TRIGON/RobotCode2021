package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.commandgroups.ShootCMDGP;
import frc.robot.commands.commandgroups.CollectCMDGP;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.motion_profiling.AutoPath;
import frc.robot.motion_profiling.TrigonSwerveControllerCMDGP;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.drivetrain.SupplierFieldDriveCMD;
import frc.robot.subsystems.intake.IntakeSS;
import frc.robot.subsystems.intakeOpener.IntakeOpenerSS;
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
    private SupplierFieldDriveCMD supplierFieldDriveCMD;

    private TrigonSwerveControllerCMDGP motionTest;
    private ShootCMDGP shootCMDGP;
    private CollectCMDGP collectCMDGP;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, true);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        subsystemContainer = new SubsystemContainerA();
        dashboardController = new DashboardController();
        xboxController = new TrigonXboxController(0);
        limelight = new PitcherLimelight(robotConstants.extendedLimelightConstants,
                robotConstants.retractedLimelightConstants, subsystemContainer.PITCHER_SS);

        initializeCommands();

        SmartDashboard.putData("Shooter Command", shooterCMD);
        SmartDashboard.putData("CalibrateShooterKfCMD", calibrateShooterKfCMD);
        SmartDashboard.putData("motion", motionTest);
    }

    /**
     * initializes all commands
     */
    public void initializeCommands() {
        SmartDashboard.putNumber("Shooter/Desired Velocity", 0);
        shooterCMD = new ShooterCMD(subsystemContainer.SHOOTER_SS, null, robotConstants.shooterConstants,
                () -> SmartDashboard.getNumber("Shooter/Desired Velocity", 0));
        calibrateShooterKfCMD = new CalibrateShooterKfCMD(subsystemContainer.SHOOTER_SS, robotConstants.shooterConstants);
        supplierFieldDriveCMD = new SupplierFieldDriveCMD(subsystemContainer.DRIVETRAIN_SS,
                () -> Math.signum(xboxController.getX(GenericHID.Hand.kRight))
                        * Math.pow(xboxController.getX(GenericHID.Hand.kRight), 2) / 7,
                () -> Math.signum(xboxController.getY(GenericHID.Hand.kRight))
                        * Math.pow(xboxController.getY(GenericHID.Hand.kRight), 2) / 7,
                () -> Math.signum(xboxController.getX(GenericHID.Hand.kLeft))
                        * Math.pow(xboxController.getX(GenericHID.Hand.kLeft), 2) / 7);

        motionTest = new TrigonSwerveControllerCMDGP(subsystemContainer.DRIVETRAIN_SS, robotConstants.motionProfilingConstants,
                AutoPath.Test);

        subsystemContainer.DRIVETRAIN_SS.setDefaultCommand(supplierFieldDriveCMD);
        calibrateShooterKfCMD = new CalibrateShooterKfCMD(subsystemContainer.SHOOTER_SS, robotConstants.shooterConstants);

        shootCMDGP = new ShootCMDGP(subsystemContainer, robotConstants, limelight);
        collectCMDGP = new CollectCMDGP(subsystemContainer, robotConstants);
    }

    public void updateDashboard() {
        dashboardController.update();
        Logger.updateEntries();
    }

    /**
     * call this method periodically
     */
    public void periodic() {
        updateDashboard();
        SmartDashboard.putNumber("Shooter/Velocity", subsystemContainer.SHOOTER_SS.getVelocityRPM());
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
