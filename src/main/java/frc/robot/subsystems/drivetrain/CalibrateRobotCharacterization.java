package frc.robot.subsystems.drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.DrivetrainConstants;

public class CalibrateRobotCharacterization extends CommandBase {
    private static final double kUpdateRate = 0.01;
    private NetworkTableEntry autoSpeedEntry;
    private NetworkTableEntry telemetryEntry;
    private NetworkTableEntry rotateEntry;
    private Number[] numberArray;
    private DrivetrainSS drivetrain;
    private DrivetrainConstants constants;

    /**
     * Calibrate feed-forward values for the motion profiling.
     * This command sends data via networktable.
     */
    public CalibrateRobotCharacterization(DrivetrainSS drivetrain, DrivetrainConstants constants) {
        this.drivetrain = drivetrain;
        this.constants = constants;
        addRequirements(drivetrain);
        autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
        telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
        rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
        numberArray = new Number[10];
    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().setUpdateRate(kUpdateRate);
    }

    @Override
    public void execute() {
        // Retrieve values to send back before telling the motors to do something
        double now = Timer.getFPGATimestamp();

        double leftPosition = constants.FRONT_LEFT_CONSTANTS.angleMotor.getSelectedSensorPosition();
        double leftRate = constants.FRONT_LEFT_CONSTANTS.angleMotor.getSelectedSensorVelocity();

        double rightPosition = drivetrain.getRightDistance();
        double rightRate = drivetrain.getRightVelocity();

        double battery = RobotController.getBatteryVoltage();

        double leftMotorVoltage = drivetrain.getLeftMotorOutputVoltage();
        double rightMotorVoltage = drivetrain.getRightMotorOutputVoltage();

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);

        // command motors to drive forward or turn
        drivetrain.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);

        // send telemetry data array back to NT
        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVoltage;
        numberArray[4] = rightMotorVoltage;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = drivetrain.getRadianAngle();

        telemetryEntry.setNumberArray(numberArray);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
        drivetrain.setRampRate(DrivetrainConstants.kRampRate);
    }
}
