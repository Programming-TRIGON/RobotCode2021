package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class SupplierDriveCMD extends CommandBase {
    private final DrivetrainSS drivetrain;
    Supplier<Double> x, y, rot;

    /**
     * Drives a drivetrain based on the given suppliers, relative to the robot
     * itself.
     *
     * @param drivetrain the drivetrain to drive
     * @param x          the supplier for the x power, between -1 and 1
     * @param y          the supplier for the y power, between -1 and 1
     * @param rot        the supplier for the rotation power, between -1 and 1
     */
    public SupplierDriveCMD(DrivetrainSS drivetrain, Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot) {
        this.drivetrain = drivetrain;
        this.x = x;
        this.y = y;
        this.rot = rot;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.powerDrive(x.get(), y.get(), rot.get());
     }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopMoving();
    }
}
