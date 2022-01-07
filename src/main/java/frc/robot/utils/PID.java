package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * PID
 */
public class PID {
    
    ShuffleboardTab tab;
    ShuffleboardLayout layout;
    String name;

    NetworkTableEntry sb_kP;
    NetworkTableEntry sb_kI;
    NetworkTableEntry sb_kD;

    double kP;
    double kI;
    double kD;

    /**
     * @param name
     *  Name of the PID to put on Shuffleboard
     * @param kP
     *  Proportinal Value of the PID
     * @param kI
     *  Integral Value of the PID
     * @param kD
     *  Derivative Value of the PID
     * 
     * <p>For more information on PIDS: https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html</p>
     */
    public PID(String name, double kP, double kI, double kD){
        tab = Shuffleboard.getTab("PID");
        this.name = name;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     *  Adds the P, I, D values on shuffleboard with the given name in the constructer
     */
    public void addToShuffleboard(){
        layout = tab.getLayout(name, BuiltInLayouts.kGrid).withSize(1, 3);

        sb_kP = layout.add("kP", kP).getEntry();
        sb_kI = layout.add("kI", kI).getEntry();
        sb_kD = layout.add("kD", kD).getEntry();
    }

    /**
     * Sets the PID values based off of what is on Shuffleboard
     */
    public void syncShuffleboard(){
        kP = sb_kP.getDouble(kP);
        kI = sb_kI.getDouble(kI);
        kD = sb_kD.getDouble(kD);
    }

    /**
     * @param constraints
     *  TrapezoidProfile.Constraints. Takes in a MaxVelocity, and MaxAcceleration
     * @return
     *  The PID as a ProfiledPIDController
     */
    public ProfiledPIDController getAsProfiledPIDController(TrapezoidProfile.Constraints constraints){
        return new ProfiledPIDController(kP, kI, kD, constraints);
    }

    /**
     * @return
     *  The PID as a PIDController
     */
    public PIDController getAsPidController(){
        return new PIDController(kP, kI, kD);
    }

    /**
     * @return
     *  The Proprtional value of the PID
     */
    public double getkP() {
        return kP;
    }

    /**
     * @return
     *  The Integral value of the PID
     */
    public double getkI() {
        return kI;
    }

    /**
     * @return
     *  The Derivative value of the PID
     */
    public double getkD() {
        return kD;
    }
    
}
