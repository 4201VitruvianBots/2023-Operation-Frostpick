package frc.robot.utils;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants;

public class LoadPath {

    PathPlannerTrajectory trajectory;    

    public LoadPath(String path){
        this.trajectory = PathPlanner.loadPath(path, Constants.MAX_VEL, Constants.MAX_ACCEL);
    }

    public PathPlannerTrajectory getTrajectory(){
        return trajectory;
    }
    
}
