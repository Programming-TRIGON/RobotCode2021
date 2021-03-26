package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.led.LedSS;
import frc.robot.subsystems.shooter.CalibrateShooterKfCMD;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.utilities.DashboardController;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private final RobotA robotConstants;
    private final HomeField fieldConstants;
    private final DashboardController dashboardController;
    private final LedSS ledSS;
    private final ShooterSS shooterSS;
    private final DrivetrainSS drivetrainSS;
    private ShooterCMD shooterCMD;
    private CalibrateShooterKfCMD calibrateShooterKfCMD;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        dashboardController = new DashboardController();
        ledSS=new LedSS(robotConstants.ledConstants);
        drivetrainSS = new DrivetrainSS(robotConstants.drivetrainConstants);
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
