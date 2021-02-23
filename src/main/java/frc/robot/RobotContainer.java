package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.drivetrain.SupplierDriveCMD;
import frc.robot.utilities.DashboardController;
import frc.robot.utilities.TrigonXboxController;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private RobotA robotConstants;
    private HomeField fieldConstants;
    private DashboardController dashboardController;
    private DrivetrainSS drivetrainSS;
    private TrigonXboxController controller;
    private SupplierDriveCMD driveCMD;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        dashboardController = new DashboardController();
        drivetrainSS = new DrivetrainSS(robotConstants.drivetrainConstants);
        controller = new TrigonXboxController(0);
    }

    /**
     * initializes all commands
     */
    public void initializeCommands() {
        driveCMD = new SupplierDriveCMD(drivetrainSS, () -> controller.getX(Hand.kRight), () -> controller.getY(Hand.kRight),
                () -> controller.getX(Hand.kLeft));
        drivetrainSS.setDefaultCommand(driveCMD);
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
    }
}
