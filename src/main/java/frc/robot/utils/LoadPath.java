package frc.robot.utils;

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
