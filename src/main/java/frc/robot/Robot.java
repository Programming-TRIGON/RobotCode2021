package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private RobotContainer robotContainer;

	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
		// robot.win=true
	}

	@Override
	public void robotPeriodic() {
		robotContainer.periodic();
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		robotContainer.autonomousInit();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		robotContainer.teleopInit();
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}
}
