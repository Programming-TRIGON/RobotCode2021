package frc.robot;

import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.utilities.DashboardController;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private RobotA robotConstants;
    private HomeField fieldConstants;
    private DashboardController dashboardController;
    private DrivetrainSS drivetrainSS;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        dashboardController = new DashboardController();
        drivetrainSS = new DrivetrainSS(robotConstants.drivetrainConstants);
    }

    /**
     * initializes all commands
     */
    public void initializeCommands() {
    }

    public void updateDashboard() {
        dashboardController.update();
        Logger.updateEntries();
    }

    /** call this method periodically */
    public void periodic() {
        updateDashboard();
    }
}
