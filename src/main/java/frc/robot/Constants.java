// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.Conversion;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Conversion.inchesToMeters(22.181999);

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Conversion.inchesToMeters(22.181999);

    /**
     * The free speed of a Falcon 500 Motor
     * 
     * In actual RPM, not ticks
     */
    public static final double FALCON_500_FREE_SPEED = 6380;
    /**
     * The gear ratio of the Swerve Module
     * 
     * From L1-L4
     */
    public static final GearRatio GEAR_RATIO = GearRatio.L2;
    /**
     * The type of Swerve Module used
     * 
     * From L1-L4
     */
    public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4_L2; 

    /**
     * CAN ID of the PigeonIMU
     */
    public static final int DRIVETRAIN_PIGEON_ID = 13;
    
    // FRONT LEFT : Florida
    public static final int FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_STEER_ENCODER = 9;
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(192.205810546875); // FIXME Measure and set front left steer offset

    // FRONT RIGHT : France
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(9.3109130859375); // FIXME Measure and set back left steer offset

    // BACK RIGHT : Railroad
    public static final int BACK_RIGHT_DRIVE_MOTOR = 5;
    public static final int BACK_RIGHT_STEER_MOTOR = 6;
    public static final int BACK_RIGHT_STEER_ENCODER = 11;
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(305.419921875); // FIXME Measure and set back right steer offset

    // BACK LEFT : Real Life
    public static final int BACK_LEFT_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_STEER_MOTOR = 8;
    public static final int BACK_LEFT_STEER_ENCODER = 12;
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(126.28784179687499); // FIXME Measure and set back left steer offset
    
    // The Max Acceleration Value for the robot [only in AUTO] in mps
    public static final double MAX_AUTO_ACCEL = 3;

    // The Max Velocity for the robot [only in AUTO]
    public static final double MAX_AUTO_VELOCITY = 4;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
     * Calculate by: Motor fre speed RPM / 60 * Drive Reduction * Wheel Diameter Meters * pi
     */
    public static final double MAX_VOLTAGE = Constants.FALCON_500_FREE_SPEED / 60.0 / MODULE_CONFIGURATION.getDriveReduction() * MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.FALCON_500_FREE_SPEED / 60.0 *
            MODULE_CONFIGURATION.getDriveReduction() *
            MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    //Constraints for the profiled angle controller
    public static final double kMaxAngularSpeedRadiansPerSecond = 2.0*Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.pow(kMaxAngularSpeedRadiansPerSecond, 2);

    public static final double kSlowMaxSpeedMetersPerSecond = 1.0;
    public static final double kSlowMaxAccelerationMetersPerSecondSquared = 3.0;

    public static final TrapezoidProfile.Constraints kThetaConstraints = new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared)

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
    
    public static final TrajectoryConfig kZeroToSlow =
        new TrajectoryConfig(kSlowMaxSpeedMetersPerSecond, kSlowMaxAccelerationMetersPerSecondSquared)
        .setKinematics(kSwerveKinematics)
        .setStartVelocity(0)
        .setEndVelocity(1.0);
}
