package com.team1816.season.auto.modes;

import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.DriveOpenLoopAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.auto.paths.DriveStraightPath;
import com.team1816.lib.autopath.AutopathAlgorithm;
import com.team1816.lib.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.Map;

public class TestAllDynamicPointsAutoMode extends AutoMode {
    ArrayList<String> failedPointNames = new ArrayList<>();

    public TestAllDynamicPointsAutoMode(){
        ArrayList<Map.Entry<String, Pose2d>> points = new ArrayList<>(robotState.dynamicAutoScript2025.getAllDynamicPoints().entrySet());
        for(Map.Entry<String, Pose2d> point1 : points){
            for(Map.Entry<String, Pose2d> point2 : points){
                if (point1.getKey().equals(point2.getKey()))
                    continue;
                else{
                    try{
                        AutopathAlgorithm.calculateAutopath(point1.getValue(), point2.getValue());
                    } catch(Exception e){
                        System.out.println(point1.getKey()+" to "+point2.getKey()+" threw an error");
                        throw e;
                    }
                    if (AutopathAlgorithm.calculateAutopath(point1.getValue(), point2.getValue()) == null)
                        failedPointNames.add("failed point: \"" + point1.getKey() + "\" to point: \"" + point2.getKey() + "\"");
                }
            }
        }
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        for(String name : failedPointNames){
            System.out.println(name);
        }

        if(failedPointNames.isEmpty()){
            System.out.println("EVERY DYNAMIC AUTO POINT WORKS !!!!!!!");
        }

        runAction(new DriveOpenLoopAction(10, .25));
    }
}
