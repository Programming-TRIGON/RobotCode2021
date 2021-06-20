package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utilities.DriverStationLogger;

public class BlinkAndLogCMD extends InstantCommand {
	private final LedSS ledSS;
	private final String message;
	private final LedBlinkColor ledBlinkColor;

	public BlinkAndLogCMD(LedSS ledSS, String message, LedBlinkColor ledBlinkColor) {
		this.ledSS = ledSS;
		this.message = message;
		this.ledBlinkColor = ledBlinkColor;
		if (ledSS != null)
			addRequirements(ledSS);
	}

	@Override
	public void initialize() {
		if (ledSS != null)
			ledSS.blinkColor(ledBlinkColor);
		DriverStationLogger.logToDS(message);
	}
}
