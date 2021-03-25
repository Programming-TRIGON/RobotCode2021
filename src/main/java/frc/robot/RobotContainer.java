package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.subsystems.drivetrain.SupplierFieldDriveCMD;
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

        supplierFieldDriveCMD = new SupplierFieldDriveCMD(
                drivetrainSS,
                () -> Math.signum(xboxController.getX(GenericHID.Hand.kRight)) * Math.pow(xboxController.getX(GenericHID.Hand.kRight), 2) / 7,
                () -> Math.signum(xboxController.getY(GenericHID.Hand.kRight)) * Math.pow(xboxController.getY(GenericHID.Hand.kRight), 2) / 7,
                () -> Math.signum(xboxController.getX(GenericHID.Hand.kLeft)) * Math.pow(xboxController.getX(GenericHID.Hand.kLeft), 2) / 7
        );

        drivetrainSS.setDefaultCommand(supplierFieldDriveCMD);
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

    /**
     * call this method periodically
     */
    public void periodic() {
        updateDashboard();
        CommandScheduler.getInstance().run();
    }
}
