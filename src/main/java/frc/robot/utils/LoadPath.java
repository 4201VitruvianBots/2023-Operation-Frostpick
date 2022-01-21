package frc.robot.utils;

import frc.robot.Constants;

public class LoadPath {

    PathPlannerTrajectory trajectory;    

    public LoadPath(String path){
        this.trajectory = PathPlanner.loadPath(path, Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_AUTO_ACCEL);
    }

    public PathPlannerTrajectory getTrajectory(){
        return trajectory;
    }
    
}
