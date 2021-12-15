package frc.robot;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.utils.PID;

public final class PIDContainer {
    
    // Declare PIDS HERE:

    // AUTO PID'S:
    public static final PID AUTO_X_PID = new PID("Auto-X PID", 0.2, 0, 0);
    public static final PID AUTO_Y_PID = new PID("Auto-Y PID", 0.2, 0, 0);
    public static final ProfiledPIDController AUTO_THETA_PID = new PID("Auto-Theta PID", 0.2, 0, 0).getAsProfiledPIDController(
        new TrapezoidProfile.Constraints(10, 11));
}
