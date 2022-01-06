package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.PID;

public final class PIDContainer {

    /**
     * PID To control the X Translation of the robot in Auto
     */
    public static final PID AUTO_X_PID = new PID("Auto-X PID", 1, 0, 0);

    /**
     * PID To control the Y Translation of the robot in Auto
     */
    public static final PID AUTO_Y_PID = new PID("Auto-Y PID", 1, 0, 0);
    
    /**
     * PID To control the Rotation of the robot in Auto
     */
    public static final ProfiledPIDController AUTO_THETA_PID = new PID("Auto-Theta PID", 1, 0, 0).getAsProfiledPIDController(
        new TrapezoidProfile.Constraints(1, 1));
}
