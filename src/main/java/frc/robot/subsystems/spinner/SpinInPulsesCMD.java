package frc.robot.subsystems.spinner;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.SpinnerConstants;

public class SpinInPulsesCMD extends CommandBase {
	private final SpinnerSS spinnerSS;
	private final SpinnerConstants constants;
	private DoubleSupplier power;
	private double output;
	private double lastTimeSpun;
	private DoubleSupplier pulseLength;
	
	public SpinInPulsesCMD(SpinnerSS spinnerSS, SpinnerConstants constants, DoubleSupplier power, DoubleSupplier pulseLength) {
		this.spinnerSS = spinnerSS;
		this.constants = constants;
		this.power = power;
		this.pulseLength = pulseLength;
		addRequirements(spinnerSS);
	}
	
	public SpinInPulsesCMD(SpinnerSS spinnerSS, SpinnerConstants constants, DoubleSupplier power) {
		this(spinnerSS, constants, power, () -> constants.PULSE_LENGTH);
	}
	
	public SpinInPulsesCMD(SpinnerSS spinnerSS, SpinnerConstants constants) {
		this(spinnerSS, constants, () -> constants.PULSE_MOTOR_POWER);
	}
	
	@Override
	public void initialize() {
		output = power.getAsDouble();
		lastTimeSpun = Timer.getFPGATimestamp();
	}
	
	@Override
	public void execute() {
		if(Timer.getFPGATimestamp() - lastTimeSpun > pulseLength.getAsDouble()) {
			output = -output;
			lastTimeSpun = Timer.getFPGATimestamp();
		}
		spinnerSS.move(output);
	}
	
	@Override
	public void end(boolean interrupted) {
		spinnerSS.move(0);
	}
	
	@Override
	public boolean isFinished() {
		return false;
	}
}
