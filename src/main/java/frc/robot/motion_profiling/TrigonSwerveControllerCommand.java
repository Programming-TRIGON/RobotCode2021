package frc.robot.motion_profiling;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.RobotConstants.MotionProfilingConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSS;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an array. The desired wheel and module
 * rotation velocities should be taken from those and used in velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes to the angle given in the final state of the trajectory.
 */
public class TrigonSwerveControllerCommand extends SequentialCommandGroup {

    public TrigonSwerveControllerCommand(DrivetrainSS drivetrainSS, MotionProfilingConstants constants, AutoPath path) {
        addCommands(
                new InstantCommand(() -> drivetrainSS.resetOdometry(path.getPath().getTrajectory().getInitialPose()),
                        drivetrainSS),
                new SwerveControllerCommand(
                    path.getPath().getTrajectory(),
                    drivetrainSS::getPose,
                    drivetrainSS.getKinematics(),
                    constants.X_PID_CONTROLLER,
                    constants.Y_PID_CONTROLLER,
                    constants.THETA_PROFILED_PID_CONTROLLER,
                    drivetrainSS::setDesiredStates, drivetrainSS),
                new InstantCommand(drivetrainSS::stopMoving, drivetrainSS));
    }
}
