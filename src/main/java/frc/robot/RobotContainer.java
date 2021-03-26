package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.fields.HomeField;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
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
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
    private final RobotA robotConstants;
    private final HomeField fieldConstants;
    private final SubsystemContainerA subsystemContainer;
    private final DashboardController dashboardController;
    private ShooterCMD shooterCMD;
    private CalibrateShooterKfCMD calibrateShooterKfCMD;

    /**
     * Add classes here
     */
    public RobotContainer() {
        Logger.configureLoggingAndConfig(this, false);
        robotConstants = new RobotA();
        fieldConstants = new HomeField();
        subsystemContainer = new SubsystemContainerA();
        dashboardController = new DashboardController();

        initializeCommands();

        SmartDashboard.putData("Shooter Command", shooterCMD);
        SmartDashboard.putData("CalibrateShooterKfCMD", calibrateShooterKfCMD);
    }

    /**
     * initializes all commands
     */
    public void initializeCommands() {
        SmartDashboard.putNumber("Shooter/Desired Velocity", 0);
        shooterCMD = new ShooterCMD(subsystemContainer.SHOOTERSS, robotConstants.shooterConstants, null,
                () -> SmartDashboard.getNumber("Shooter/Desired Velocity", 0));
        calibrateShooterKfCMD = new CalibrateShooterKfCMD(subsystemContainer.SHOOTERSS, robotConstants.shooterConstants);
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
        SmartDashboard.putNumber("Shooter/Velocity", subsystemContainer.SHOOTERSS.getVelocityRPM());
    }

    public class SubsystemContainerA extends SubsytemContainer {
        public SubsystemContainerA() {
            LEDSS = new LedSS(robotConstants.ledConstants);
            DRIVETRAINSS = new DrivetrainSS(robotConstants.drivetrainConstants);
            SHOOTERSS = new ShooterSS(robotConstants.shooterConstants);
            PITCHERSS = new PitcherSS(robotConstants.pitcherConstants);
            LOADERSS = new LoaderSS(robotConstants.loaderConstants);
            SPINNERSS = new SpinnerSS(robotConstants.spinnerConstants);
            INTAKESS = new IntakeSS(robotConstants.intakeConstants);
            INTAKEOPENERSS = new IntakeOpenerSS(robotConstants.intakeOpenerConstants);
        }
    }
}
