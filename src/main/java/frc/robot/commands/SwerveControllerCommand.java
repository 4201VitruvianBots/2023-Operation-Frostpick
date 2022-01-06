package frc.robot.commands;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
// edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.*;

public class SwerveControllerCommand extends CommandBase{
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final Supplier<Rotation2d> m_desiredRotation;

    /**
     Constructs a new SwerveControllerCommand that when executed will follow the provided
     trajectory. This command will not return output voltages but rather raw module states from the
     position controllers which need to be put into a velocity PID.

     <p>Note: The controllers will not set the outputVolts to zero upon completion of the path-
     this is left to the user, since it is not appropriate for paths with nonstationary endstates.

     @param trajectory The trajectory to follow.
     @param pose A function that supplies the robot pose - use one of the odometry classes to
         provide this.
     @param kinematics The kinematics for the robot drivetrain.
     @param xController The Trajectory Tracker PID controller for the robot's x position.
     @param yController The Trajectory Tracker PID controller for the robot's y position.
     @param thetaController The Trajectory Tracker PID controller for angle for the robot.
     @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
         time step.
     @param outputModuleStates The raw output module states from the position controllers.
     @param requirements The subsystems to require.
    */
    @SuppressWarnings("ParameterName")
    public SwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {

        m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
        m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
        m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");

        m_controller =
            new HolonomicDriveController(
                requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
                requireNonNullParam(yController, "xController", "SwerveControllerCommand"),
                requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand"));

        m_outputModuleStates =
            requireNonNullParam(outputModuleStates, "frontLeftOutput", "SwerveControllerCommand");

        m_desiredRotation =
            requireNonNullParam(desiredRotation, "desiredRotation", "SwerveControllerCommand");

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double currentTime = m_timer.get();
        // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", 8, 5);

        // Sample the state of the path at 1.2 seconds
        // To access PathPlanner specific information, such as holonomic rotation, the state must be cast to a PathPlannerState
        PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

        // Print the holonomic rotation at the sampled time
        System.out.println(exampleState.holonomicRotation.getDegrees());

    }
}
