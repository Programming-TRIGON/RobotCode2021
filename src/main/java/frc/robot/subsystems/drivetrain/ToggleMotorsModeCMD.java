package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ToggleMotorsModeCMD extends CommandBase {
    private final DrivetrainSS drivetrainSS;
    private boolean onBrake;

    public ToggleMotorsModeCMD(DrivetrainSS drivetrainSS) {
        this.drivetrainSS = drivetrainSS;
        onBrake = true;
    }

    @Override
    public void initialize() {
        drivetrainSS.setMotorsMode(onBrake ? NeutralMode.Coast : NeutralMode.Brake);
        System.out.println("onBrake:\t" + onBrake);
        onBrake = !onBrake;

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
