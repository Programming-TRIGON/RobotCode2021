package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.OverrideCommand;
import frc.robot.commands.command_groups.BackupAuto;
import frc.robot.commands.command_groups.ShootCMDGP;
import frc.robot.commands.command_groups.ShootWithoutLimelight;
import frc.robot.constants.robots.RobotA;
import frc.robot.subsystems.shooter.ShooterCMD;
import frc.robot.utilities.TrigonXboxController;
import frc.robot.vision.Target;
import frc.robot.vision.limelights.PitcherLimelight;
import io.github.oblarg.oblog.Logger;

public class RobotContainer {
	private static final int DRIVER_XBOX_CONTROLLER_PORT = 0;
	private static final int OPERATOR_XBOX_CONTROLLER_PORT = 1;
	
	private final RobotA robotConstants;
	private final SubsystemContainerA subsystemContainer;
	private final TrigonXboxController driverXboxController;
	private final TrigonXboxController overrideXboxController;
	private final PitcherLimelight limelight;
	private final CommandContainer commandContainer;
	private final DashboardDataContainer dashboardDataContainer;
	
	/**
	* Add classes here
	*/
	public RobotContainer() {
		Logger.configureLoggingAndConfig(this, true);
		robotConstants = new RobotA();
		subsystemContainer = new SubsystemContainerA(robotConstants);
		driverXboxController = new TrigonXboxController(DRIVER_XBOX_CONTROLLER_PORT);
		overrideXboxController = new TrigonXboxController(OPERATOR_XBOX_CONTROLLER_PORT);
		limelight = new PitcherLimelight(robotConstants.extendedLimelightConstants,
		robotConstants.retractedLimelightConstants, subsystemContainer.PITCHER_SS);
		commandContainer = new CommandContainer(subsystemContainer, robotConstants, limelight, driverXboxController);
		dashboardDataContainer = new DashboardDataContainer(subsystemContainer, robotConstants, limelight, driverXboxController, commandContainer);
		
		subsystemContainer.DRIVETRAIN_SS.setDefaultCommand(commandContainer.SUPPLIER_FIELD_DRIVE_CMD);
		bindDriverCommands();
		bindOverrideCommands();
		
		Logger.configureLogging(subsystemContainer.DRIVETRAIN_SS);
		limelight.startVision(Target.PowerPort);
	}
	
	/**
	* Binds all commands to the buttons that use them. Call this after initializing
	* the commands.
	*/
	public void bindDriverCommands() {
		driverXboxController.getRightBumper().whenHeld(commandContainer.COLLECT_CMDGP).whenReleased(commandContainer.OPEN_INTAKE_CMD);
		driverXboxController.getLeftBumper().whenPressed(commandContainer.CHANGE_DRIVE_ROTATION);
		driverXboxController.getButtonY().whenPressed(commandContainer.RESET_DIRECTION);
		driverXboxController.getButtonX().whenPressed(
		new InstantCommand(subsystemContainer.PITCHER_SS::toggleSolenoid, subsystemContainer.PITCHER_SS));
		driverXboxController.getButtonB().whileHeld(commandContainer.SHOOT_CMDGP.withInterrupt(this::cancelShooterCMD));
		
		// robot.win=true
	}
	
	public void bindOverrideCommands() {
		overrideXboxController.getButtonA().whenPressed(commandContainer.TOGGLE_PITCHER);
		overrideXboxController.getButtonX().whenHeld(new ShooterCMD(subsystemContainer.SHOOTER_SS, null, robotConstants.shooterConstants, () -> 3200));
		overrideXboxController.getButtonB().whileHeld(new ShootWithoutLimelight(subsystemContainer, robotConstants, () -> 3500)).whenReleased(commandContainer.CLOSE_PITCHER);
		overrideXboxController.getButtonY().whileHeld(new ShootCMDGP(subsystemContainer, robotConstants,limelight,  3500));
		overrideXboxController.getLeftStickButton().whileHeld(new OverrideCommand(subsystemContainer.LOADER_SS, () -> overrideXboxController.getY(Hand.kLeft)));
		overrideXboxController.getRightStickButton().whileHeld(new OverrideCommand(subsystemContainer.SPINNER_SS, () -> overrideXboxController.getX(Hand.kRight)));
		overrideXboxController.getRightBumper().whileHeld(commandContainer.OPEN_LIFT_CMD);
		overrideXboxController.getLeftBumper().whileHeld(commandContainer.CLOSE_LIFT_CMD);
		overrideXboxController.getBackXboxButton().whileHeld(commandContainer.WINCH_CMD);
	}
	
	/**
	* Checks the values of the driving joysticks and if one of them is above a
	* specified threshold. This is done incase the driver desires to continue //
	* moving before the robot is finished shooting.
	*/
	private boolean cancelShooterCMD() {
		double threshold = robotConstants.shooterConstants.CANCEL_CMDGP_AXIS_THRESHOLD;
		return Math.abs(driverXboxController.getX(Hand.kRight)) >= threshold
		|| Math.abs(driverXboxController.getY(Hand.kRight)) >= threshold
		|| Math.abs(driverXboxController.getX(Hand.kLeft)) >= threshold;
	}
	
	public void updateDashboard() {
		dashboardDataContainer.updateDashboard();
		Logger.updateEntries();
	}
	
	/**
	* Call this method in the autonomousInit.
	*/
	public void autonomousInit() {
		CommandScheduler.getInstance().schedule(commandContainer.OPEN_INTAKE_CMD);
		CommandScheduler.getInstance().schedule(new BackupAuto(subsystemContainer, robotConstants, limelight));
	}
	
	/**
	* Call this method in the teleopInit.
	*/
	public void teleopInit() {
		CommandScheduler.getInstance().schedule(commandContainer.OPEN_INTAKE_CMD);
	}
	
	/**
	* Call this method periodically.
	*/
	public void periodic() {
		updateDashboard();
	}
}
