package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.constants.RobotConstants;
import frc.robot.utilities.DashboardController;
import frc.robot.vision.limelights.PitcherLimelight;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.*;

/**
 * DashboardDataContainer contains all the data to be viewed or put in the
 * dashboard. UpdateDashboard should be called periodicly.
 */
public class DashboardDataContainer {
    DashboardController dashboardController;

    public DashboardDataContainer(SubsystemContainer subsystemContainer, RobotConstants robotConstants,
            PitcherLimelight limelight, GenericHID driverController, CommandContainer container) {
        dashboardController = new DashboardController();

        // CMDGP
        putData("Shoot without pitcher CMDGP", container.SHOOT_CMDGP);
        putData(" shoot ", container.SHOOT_WITH_PITCHER_CMDGP);
        putData(" collect ", container.COLLECT_CMDGP);

        // Calibration
        putData("CalibrateShooterKfCMD", container.CALIBRATE_SHOOTER_KF_CMD);
        putData("CalibrateLoaderKfCMD", container.CALIBRATE_LOADER_KF_CMD);

        // Shooter
        putData("Shooter Command", container.SHOOTER_CMD);

        // Drivetrain
        putData("TurnToTargetCMD", container.TURN_TO_TARGET_CMD);
        putData("TurnAndPositionToTargetCMD", container.TURN_AND_POSITION_TO_TARGET_CMD);
        putData("Drivetrain/ResetDirection", container.RESET_DIRECTION);
        putData("ToggleMotorsModeCMD", container.TOGGLE_DRIVETRAIN_MOTORS_NEUTRAL_MODE_CMD);
        putData("TrigonSwerveControllerCMDGP", container.MOTION_TEST);

        // Pitcher
        putData("toggle solenoid", container.TOGGLE_PITCHER);
        putData("Pitcher/Open", container.OPEN_PITCHER);
        putData("Pitcher/Close", container.CLOSE_PITCHER);

        // Loader
        putData("Loader/Load", container.LOADER_CMD);

        // DashboardController
        dashboardController.addBoolean("Pitcher/State", subsystemContainer.PITCHER_SS::getSolenoidState);
        dashboardController.addNumber("Shooter/Velocity", subsystemContainer.SHOOTER_SS::getVelocityRPM);
        dashboardController.addNumber("Loader/Velocity", subsystemContainer.LOADER_SS::getVelocity);
//        dashboardController.addNumber("PitcherLimelight/distance", limelight::calculateDistanceFromTower);
        dashboardController.addNumber("Xbox/X", driverController::getX);
        dashboardController.addNumber("Xbox/Y", driverController::getY);
        dashboardController.addBoolean("Limelight/isExtended", limelight::hoodExtended);
        dashboardController.addBoolean("Limelight/hasTarget", limelight::hasTarget);
    }

    public void updateDashboard() {
        dashboardController.update();
    }
}
