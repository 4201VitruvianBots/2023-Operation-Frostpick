package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

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

    public PID(String name, double kP, double kI, double kD){
        tab = Shuffleboard.getTab("PID");
        this.name = name;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void addToShuffleboard(){
        layout = tab.getLayout(name, BuiltInLayouts.kList);

        sb_kP = layout.add("kP", 0).getEntry();
        sb_kI = layout.add("kI", 0).getEntry();
        sb_kD = layout.add("kD", 0).getEntry();
    }

    public void syncShuffleboard(){
        kP = sb_kP.getDouble(0);
        kI = sb_kI.getDouble(0);
        kD = sb_kD.getDouble(0);
    }

    public ProfiledPIDController getAsProfiledPIDController(TrapezoidProfile.Constraints constraints){
        return new ProfiledPIDController(kP, kI, kD, constraints);
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }
    
}
