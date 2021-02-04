package frc.robot.subsystems.loader;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants;

public class LoaderCMD extends CommandBase {
    private final LoaderSS loaderSS;
    private final RobotConstants.LoaderConstants constants;
    private final DoubleSupplier velocity;

    public LoaderCMD(LoaderSS loaderSS, RobotConstants.LoaderConstants constants, DoubleSupplier velocity) {
        this.loaderSS = loaderSS;
        this.constants = constants;
        this.velocity = velocity;
        addRequirements(loaderSS);
    }

    public LoaderCMD(LoaderSS loaderSS, RobotConstants.LoaderConstants constants, double velocity) {
        this(loaderSS, constants, () -> velocity);
    }

    public LoaderCMD(LoaderSS loaderSS, RobotConstants.LoaderConstants constants) {
        this(loaderSS, constants, () -> constants.DEFAULT_VELOCITY);
    }

    @Override
    public void execute() {
        loaderSS.setVelocity(velocity.getAsDouble());
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
