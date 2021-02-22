public class SpinerSS extends OverridableSubsystem {
  private TrigonTalonSRX motor;
  private RobotConstants.SpinerConstants constants;

  public SpinerSS(RobotConstants.SpinerConstants constants) {
      this.constants = constants;
     motor = constants.CAN_MAP.MOTOR;
      motor.setInverted(constants.IS_INVERTED);
    }
     /**
    *
    * @param power to be set to the motor (between -1 and 1)
    */
    public void overriddenMove(double power) {
     motor.set(power);
  }
}