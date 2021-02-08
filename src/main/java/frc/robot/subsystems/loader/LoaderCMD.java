package frc.robot.subsystems.loader;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.LoaderConstants;

public class LoaderCMD extends CommandBase {
    private final LoaderSS loaderSS;
    private final LoaderConstants constants;
    private final DoubleSupplier velocity;

    public LoaderCMD(LoaderSS loaderSS, LoaderConstants constants, DoubleSupplier velocity) {
        this.loaderSS = loaderSS;
        this.constants = constants;
        this.velocity = velocity;
        addRequirements(loaderSS);
    }

    public LoaderCMD(LoaderSS loaderSS, LoaderConstants constants, double velocity) {
        this(loaderSS, constants, () -> velocity);
    }

    public LoaderCMD(LoaderSS loaderSS, LoaderConstants constants) {
        this(loaderSS, constants, () -> constants.DEFAULT_SHOOTING_VELOCITY);
    }

    @Override
    public void execute() {
        loaderSS.setDesiredVelocity(velocity.getAsDouble());
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
