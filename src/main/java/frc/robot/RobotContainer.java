package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.drivetrain.SupplierFieldDriveCMD;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.led.LedSS;
import frc.robot.subsystems.shooter.CalibrateShooterKfCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.utilities.DashboardController;
import frc.robot.utilities.TrigonXboxController;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private RobotA robotConstants;
    private HomeField fieldConstants;
    private DashboardController dashboardController;
    private DrivetrainSS drivetrainSS;
    private SupplierFieldDriveCMD supplierFieldDriveCMD;
    private TrigonXboxController xboxController;
    private final LedSS ledSS;
    private final ShooterSS shooterSS;
    
    private ShooterCMD shooterCMD;
    private CalibrateShooterKfCMD calibrateShooterKfCMD;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, true);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        dashboardController = new DashboardController();
        xboxController = new TrigonXboxController(0);
        drivetrainSS = new DrivetrainSS(robotConstants.drivetrainConstants);
        ledSS=new LedSS(robotConstants.ledConstants);
        shooterSS = new ShooterSS(robotConstants.shooterConstants);

        initializeCommands();

        SmartDashboard.putData("Shooter Command", shooterCMD);
        SmartDashboard.putData("CalibrateShooterKfCMD" , calibrateShooterKfCMD);
    }

    /**
     * initializes all commands
     */
    public void initializeCommands() {
        SmartDashboard.putNumber("Shooter/Desired Velocity", 0);
        shooterCMD = new ShooterCMD(shooterSS, robotConstants.shooterConstants, null,
                () -> SmartDashboard.getNumber("Shooter/Desired Velocity", 0));
        calibrateShooterKfCMD = new CalibrateShooterKfCMD(shooterSS, robotConstants.shooterConstants);
        supplierFieldDriveCMD = new SupplierFieldDriveCMD(
                drivetrainSS,
                () -> Math.signum(xboxController.getX(GenericHID.Hand.kRight)) * Math.pow(xboxController.getX(GenericHID.Hand.kRight), 2) / 7,
                () -> Math.signum(xboxController.getY(GenericHID.Hand.kRight)) * Math.pow(xboxController.getY(GenericHID.Hand.kRight), 2) / 7,
                () -> Math.signum(xboxController.getX(GenericHID.Hand.kLeft)) * Math.pow(xboxController.getX(GenericHID.Hand.kLeft), 2) / 7
        );
        
        drivetrainSS.setDefaultCommand(supplierFieldDriveCMD);
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
        SmartDashboard.putNumber("Shooter/Velocity", shooterSS.getVelocityRPM());
    }
}
