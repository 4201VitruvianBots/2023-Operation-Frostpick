package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.PID;

public class PIDContainer {

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
    public static final PID AUTO_THETA_PID_CONTROLLER = new PID("Auto-Theta PID", 1, 0, 0);
    public static final ProfiledPIDController AUTO_THETA_PID = AUTO_THETA_PID_CONTROLLER.getAsProfiledPIDController(
        new TrapezoidProfile.Constraints(1, 1));

    public PIDContainer(){
        AUTO_X_PID.addToShuffleboard();
        AUTO_Y_PID.addToShuffleboard();
        AUTO_THETA_PID_CONTROLLER.addToShuffleboard();
    }

    public void periodic(){
        AUTO_X_PID.syncShuffleboard();
        AUTO_Y_PID.syncShuffleboard();
        AUTO_THETA_PID_CONTROLLER.syncShuffleboard();
    }
}
