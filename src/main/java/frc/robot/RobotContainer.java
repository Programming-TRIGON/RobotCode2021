package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    private SupplierDriveCMD supplierDriveCMD;
    private TrigonXboxController xboxController;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        dashboardController = new DashboardController();
        xboxController = new TrigonXboxController(0);
        drivetrainSS = new DrivetrainSS(robotConstants.drivetrainConstants);

        supplierDriveCMD = new SupplierDriveCMD(
                drivetrainSS,
                () -> xboxController.getX(GenericHID.Hand.kRight)/4,
                () -> xboxController.getY(GenericHID.Hand.kRight)/4,
                () -> xboxController.getX(GenericHID.Hand.kLeft)/4
        );
        drivetrainSS.setDefaultCommand(supplierDriveCMD);
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
