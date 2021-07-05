package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class WinchSS extends OverridableSubsystem {
    private final ClimberConstants constants;
    private final TrigonTalonSRX masterMotor;
    private final TrigonTalonSRX rightMotor;
    private final TrigonTalonSRX leftMotor;

    public WinchSS(ClimberConstants constants) {
        this.constants = constants;
        rightMotor = constants.CAN_MAP.RIGHT_WINCH_MOTOR;
        leftMotor = constants.CAN_MAP.LEFT_WINCH_MOTOR;
        masterMotor = rightMotor;

        rightMotor.follow(masterMotor);
        leftMotor.follow(masterMotor);
    }

    public void overriddenMove(double power) {
        masterMotor.set(power);
    }
}
