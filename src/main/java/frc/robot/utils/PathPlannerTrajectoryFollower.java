package frc.robot.utils;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class PathPlannerTrajectoryFollower {
    private SwerveDriveOdometry m_odometry;
    private Trajectory m_trajectory;
    private Field2d m_field2d = new Field2d();
    private Rotation2d m_finalRotation = new Rotation2d();
    private HolonomicDriveController m_holoController;
    private double m_tolerance = 0.1;

    public PathPlannerTrajectoryFollower() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCEL);

        ProfiledPIDController rotationalController = new ProfiledPIDController(0.9, 0, 0, constraints);
        rotationalController.enableContinuousInput(-Math.PI, Math.PI);
        m_odometry = new SwerveDriveOdometry(DrivetrainSubsystem.getInstance().getKinematics(), DrivetrainSubsystem.getInstance().getGyroscopeRotation());

        m_holoController = DrivetrainSubsystem.getInstance().getHolonomicController();
    }
    
    public void follow(double time) {
        Trajectory.State reference = m_trajectory.sample(time);

        Pose2d current = DrivetrainSubsystem.getInstance().getCurrentPose();
        m_field2d.setRobotPose(reference.poseMeters);
        ChassisSpeeds output = m_holoController.calculate(current, reference.poseMeters, reference.velocityMetersPerSecond, m_finalRotation);
        DrivetrainSubsystem.getInstance().drive(output);
    }

    public boolean followDone() {
        Pose2d currentPose = DrivetrainSubsystem.getInstance().getCurrentPose();
		Pose2d endPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;

		return endPose.minus(currentPose).getTranslation().getNorm() < m_tolerance;

    }
}
