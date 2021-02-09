package frc.robot.subsystems.loader;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.LoaderConstants;

public class LoaderCMD extends CommandBase {
    private final LoaderSS loaderSS;
    private final LoaderConstants constants;
    private final DoubleSupplier desiredVelocity;

    public LoaderCMD(LoaderSS loaderSS, LoaderConstants constants, DoubleSupplier desiredVelocity) {
        this.loaderSS = loaderSS;
        this.constants = constants;
        this.desiredVelocity = desiredVelocity;
        addRequirements(loaderSS);
    }

    public LoaderCMD(LoaderSS loaderSS, LoaderConstants constants, double desiredVelocity) {
        this(loaderSS, constants, () -> desiredVelocity);
    }

    /*
     * Constructs an instance of LoaderCMD using the default loader velocity
     */
    public LoaderCMD(LoaderSS loaderSS, LoaderConstants constants) {
        this(loaderSS, constants, () -> constants.DEFAULT_SHOOTING_VELOCITY);
    }

    @Override
    public void execute() {
        loaderSS.setDesiredVelocity(desiredVelocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        loaderSS.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
