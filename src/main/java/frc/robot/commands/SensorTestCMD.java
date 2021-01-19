package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDColor;
import frc.robot.utilities.DriverStationLogger;

import java.util.HashMap;

public class SensorTestCMD extends CommandBase {

	private HashMap<String, TestableSubsystem> subsystems;
	private HashMap<String, double[]> initialValues;
	private double initialTime;
	private RobotConstants.TesterConstants constants;
	private LED led;

	public SensorTestCMD(RobotConstants.TesterConstants constants, LED led) {
		addRequirements(led);
		this.led = led;
		subsystems = new HashMap<>();
		initialValues = new HashMap<>();
	}

	public SensorTestCMD addSubsystem(TestableSubsystem ss, String name) {
		subsystems.put(name, ss);
		addRequirements(ss);
		return this;
	}

	@Override
	public void initialize() {
		for (String name : subsystems.keySet()) {
			TestableSubsystem ss = subsystems.get(name);
			initialValues.put(name, ss.getValues());
			ss.move(constants.MOVE_POWER);
		}
		initialTime = Timer.getFPGATimestamp();
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() >= initialTime + constants.SECONDS_TO_WAIT;
	}

	@Override
	public void end(boolean interrupted) {
		boolean allSuccess = true;
		for (String name : subsystems.keySet()) {
			TestableSubsystem ss = subsystems.get(name);
			ss.stopMoving();
			double[] values = ss.getValues();
			boolean success = true;
			for (int i = 0; i < values.length; i++) {
				// checks if the sensor values have changed at all
				if (values[i] == initialValues.get(name)[i]) {
					DriverStationLogger.logErrorToDS(name + ", Sensor number: " + i + 1 + " is not working");
					success = false;
					allSuccess = false;
				}
				// checks if the sensor is inverted
				boolean isSensorInverted = (values[i] - initialValues.get(name)[i]) * constants.MOVE_POWER > 0;
				DriverStationLogger.logToDS(
						name + ", Sensor number: " + i + 1 + ((isSensorInverted) ? "" : " not") + " is inverted");
			}
			if (success)
				DriverStationLogger.logToDS(name + " is working");
		}
		led.blinkColor(allSuccess ? LEDColor.Green : LEDColor.Red, constants.LED_BLINK_AMOUNT);
	}
}
