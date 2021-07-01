package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.led.LedSS;
import frc.robot.constants.RobotConstants.IntakeConstants;

public class IntakeCMD extends CommandBase {
    private final IntakeSS intakeSS;
    private final IntakeConstants constants;
    private final LedSS ledSS;
    private final DoubleSupplier power;

    public IntakeCMD(IntakeSS intakeSS, LedSS ledSS, IntakeConstants constants, DoubleSupplier power) {
        this.intakeSS = intakeSS;
        this.constants = constants;
        this.ledSS = ledSS;
        this.power = power;
        addRequirements(intakeSS);
    }

    public IntakeCMD(IntakeSS intakeSS, LedSS ledSS, IntakeConstants constants, double power) {
        this(intakeSS, ledSS, constants, () -> power);
    }

    public IntakeCMD(IntakeSS intakeSS, LedSS ledSS, IntakeConstants constants) {
        this(intakeSS, ledSS, constants, () -> constants.DEFAULT_MOTOR_POWER);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSS.move(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        if(ledSS != null)
            ledSS.turnOffLED();
        intakeSS.stopMoving();
    }
}
