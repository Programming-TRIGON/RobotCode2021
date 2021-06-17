package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.components.TBHController;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.utilities.Logger;

public class CalibrateShooterKfCMD extends CommandBase {

    private final ShooterSS shooterSS;
    private final ShooterConstants constants;
    private final TBHController tbhController;
    private final SimpleMotorFeedforward feedforward;
    private double desiredVelocity;
    private double endVelocity;
    private boolean postTest;
    private double lastVelocity;
    private double outputSum;
    private int sampleCount;
    private Logger logger;

    /**
     * This command autonomously runs tests and outputs a csv file including the Kf
     * value at different velocities this is used to find the optimal kF coefs for
     * calculate the correct voltage for a given velocity in RPM
     */
    public CalibrateShooterKfCMD(ShooterSS shooterSS, ShooterConstants constants) {
        addRequirements(shooterSS);

        this.shooterSS = shooterSS;
        this.constants = constants;
        this.tbhController = new TBHController(constants.TBH_CONTROLLER);
        this.feedforward = constants.SIMPLE_MOTOR_FEEDFORWARD;
        desiredVelocity = 0;
        endVelocity = 1;
    }

    @Override
    public void initialize() {
        this.logger = new Logger("ShooterKFCalibration", "Velocity", "Voltage", "SimpleMotorFeedForward Voltage");
        desiredVelocity = constants.KF_TESTING_INITIAL_DESIRED_VELOCITY;
        endVelocity = constants.KF_TESTING_INITIAL_DESIRED_VELOCITY
                + constants.KF_TESTING_VELOCITY_ACCELERATION_PER_TEST * constants.KF_TESTING_TEST_AMOUNT;
        shooterSS.setRampRate(constants.SHOOTING_RAMP_RATE);
        postTest = false;
        lastVelocity = shooterSS.getVelocityRPM();

        tbhController.setSetpoint(desiredVelocity);
        tbhController.reset();
    }

    @Override
    public void execute() {
        System.out.println(postTest);
        if (postTest) {
            shooterSS.setRampRate(constants.RIGHT_MOTOR_CONFIG.getRampRate());
            shooterSS.stopMoving();
            if (shooterSS.getVelocityRPM() == 0) {
                shooterSS.setRampRate(constants.SHOOTING_RAMP_RATE);
                desiredVelocity += constants.KF_TESTING_VELOCITY_ACCELERATION_PER_TEST;
                tbhController.setSetpoint(desiredVelocity);
                tbhController.reset();
                postTest = false;
            }
        } else {

            double output = tbhController.calculate(shooterSS.getVelocityRPM()) + constants.KF_COEF_A * desiredVelocity
                    + constants.KF_COEF_B;
            shooterSS.move(output);
            if (atSetpoint()) {
                outputSum += output;
                sampleCount++;

            } else {
                outputSum = 0;
                sampleCount = 0;
            }
            if (sampleCount == constants.KF_TESTING_CALCULATION_SAMPLE_AMOUNT) {
                double newF = outputSum / sampleCount;
                logger.log(desiredVelocity, newF, feedforward.calculate(desiredVelocity / 60));
                postTest = true;
            }
        }

        lastVelocity = shooterSS.getVelocityRPM();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSS.setRampRate(constants.RIGHT_MOTOR_CONFIG.getRampRate());
        shooterSS.stopMoving();
        logger.close();
    }

    @Override
    public boolean isFinished() {
        return desiredVelocity >= endVelocity;
    }

    public boolean atSetpoint() {
        return Math.abs(desiredVelocity - shooterSS.getVelocityRPM()) < constants.KF_TESTING_TOLERANCE
                && shooterSS.getVelocityRPM() - lastVelocity < constants.KF_TESTING_DELTA_TOLERANCE;
    }
}
