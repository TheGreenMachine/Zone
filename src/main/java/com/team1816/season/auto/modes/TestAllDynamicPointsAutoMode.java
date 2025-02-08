package com.team1816.season.auto.modes;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.DriveOpenLoopAction;
import com.team1816.lib.auto.modes.AutoMode;
import com.team1816.lib.autopath.Autopath;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.Map;

public class TestAllDynamicPointsAutoMode extends AutoMode {
    Autopath autopather;

    ArrayList<String> failedPointNames = new ArrayList<>();

    public TestAllDynamicPointsAutoMode(){
        autopather = Injector.get(Autopath.class);

        ArrayList<Map.Entry<String, Pose2d>> points = new ArrayList<>(robotState.dAllDynamicPoints.entrySet());
        for(Map.Entry<String, Pose2d> point1 : points){
            for(Map.Entry<String, Pose2d> point2 : points){
                if (point1.getKey().equals(point2.getKey()))
                    continue;
                else{
                    try{
                        autopather.calculateAutopath(point1.getValue(), point2.getValue());
                    } catch(Exception e){
                        System.out.println(point1.getKey()+" to "+point2.getKey()+" threw an error");
                        throw e;
                    }
                    if (autopather.calculateAutopath(point1.getValue(), point2.getValue()) == null)
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
