package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SwerveControllerCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;


public class TestDrive extends AutoRoutine {
    private final DrivetrainSubsystem s_dt = DrivetrainSubsystem.getInstance();

    @Override
    public void generate() {
        Trajectory straightTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1.5, 0)),
                new Pose2d(3, 0, new Rotation2d(0)), Constants.kZeroToSlow);
        
        var thetaController = new ProfiledPIDController(2.5, 0, 0, Constants.kThetaConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        

        SwerveControllerCommand straightTrajectoryCmd = new SwerveControllerCommand(straightTrajectory,
                s_dt::getCurrentPose, Constants.kSwerveKinematics,
                new PIDController(1.0, 0, 0),
                new PIDController(1.0, 0, 0), thetaController,
                s_dt::actuateModules);

        var testRun = straightTrajectoryCmd.withTimeout(10);

        testRun.schedule();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();        
    }
    
}
