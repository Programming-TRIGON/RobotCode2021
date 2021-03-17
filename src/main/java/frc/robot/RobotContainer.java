package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.led.LedSS;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.subsystems.shooter.ShooterSS;
import frc.robot.utilities.DashboardController;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private RobotA robotConstants;
    private HomeField fieldConstants;
    private DashboardController dashboardController;
    private ShooterSS shooterSS;
    private LedSS ledSS;
    private ShooterCMD shooterCMD;
    // private DrivetrainSS drivetrainSS;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        dashboardController = new DashboardController();
        // drivetrainSS = new DrivetrainSS(robotConstants.drivetrainConstants);
        shooterSS = new ShooterSS(robotConstants.shooterConstants);

        initializeCommands();

        SmartDashboard.putData("Shooter Command", shooterCMD);
    }

    /**
     * initializes all commands
     */
    public void initializeCommands() {
        SmartDashboard.putNumber("Shooter/Desired Velocity", 0);
        shooterCMD = new ShooterCMD(shooterSS, robotConstants.shooterConstants, null,
                () -> SmartDashboard.getNumber("Shooter/Desired Velocity", 0));
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
        SmartDashboard.putNumber("Shooter/Velocity", shooterSS.getVelocity());
    }
}
