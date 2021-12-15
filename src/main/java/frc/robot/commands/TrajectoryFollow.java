package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TrajectoryFollow {

  public SequentialCommandGroup getTrajectoryCommand(DrivetrainSubsystem drivetrain, Trajectory trajectory) {

    // Creates a new PID Controller to control the X position of the Robot
    PIDController xController = Constants.AUTO_X_PID.getAsPidController();
    // Creates a new PID Controller to control the Y Position of the Robot
    PIDController yController = Constants.AUTO_Y_PID.getAsPidController();
    // Creates a new PID Controller to control the angle of the robot, with Max Velocity and Max Acceleration constraints
    ProfiledPIDController thetaController = Constants.AUTO_THETA_PID;
    // Makes sure that the PID outputs values from -180 to 180 degrees
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Creates a new SwerveControllerCommand
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            // The trajectory to follow
            trajectory,
            // A method refrence for constantly getting current position of the robot
            drivetrain::getCurrentPose,
            // Getting the kinematics from the drivetrain
            drivetrain.getKinematics(),
            // Position controllers
            xController,
            yController,
            thetaController,
            // A method refrence for setting the state of the modules
            drivetrain::actuateModules,
            // Requirment of a drivetrain subsystem
            drivetrain);

    // Reset odometry to the starting pose of the trajectory. This effectively transforms the trajectory to the current pose of the robot
    drivetrain.resetOdometry(trajectory.getInitialPose());
  
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0)));
  }
}