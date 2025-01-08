package com.team1816.season;

import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.paths.AutoPath;
import com.team1816.lib.autopath.AutopathAlgorithm;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class DynamicAutoScript2025 {
    private ArrayList<SendableChooser<Pose2d>> trajectoryActions = new ArrayList<>();
    private ArrayList<TrajectoryAction> autoTrajectoryActions = new ArrayList<>();
    private Pose2d startPos = new Pose2d(new Translation2d(2,1), new Rotation2d());
    private Pose2d currentStartPos = startPos;

    public DynamicAutoScript2025(int numOfTrajectoryActions){
        for(int i = 0; i < numOfTrajectoryActions; i++)
            trajectoryActions.add(new SendableChooser<>());

        addDefaultTrajectoryOption("somewhere", new Pose2d(new Translation2d(3,1), new Rotation2d()));
        addTrajectoryOption("elsewhere", new Pose2d(new Translation2d(5,1), new Rotation2d()));
        addTrajectoryOption("narnia", new Pose2d(new Translation2d(2,5), new Rotation2d()));
        addTrajectoryOption("space", new Pose2d(new Translation2d(14,4), new Rotation2d()));

        addToSmartDashboard();
    }

    public void updateSendableChoosers(){
        ArrayList<TrajectoryAction> newAutoTrajectoryActions = new ArrayList<>();
        currentStartPos = startPos;
        for(SendableChooser<Pose2d> trajectoryAction : trajectoryActions) {
            Trajectory newTraj = AutopathAlgorithm.calculateAutopath(currentStartPos, trajectoryAction.getSelected());
            assert newTraj != null;
            if(newTraj.getStates().isEmpty())
                continue;
            newAutoTrajectoryActions.add(new TrajectoryAction(newTraj, List.of(Rotation2d.fromDegrees(0))));
            currentStartPos = trajectoryAction.getSelected();
        }
        autoTrajectoryActions = newAutoTrajectoryActions;
    }

    public ArrayList<TrajectoryAction> getAutoTrajectoryActions(){
        for(TrajectoryAction action : autoTrajectoryActions){
            System.out.println(action.getTrajectory());
        }
        return autoTrajectoryActions;
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    private void addTrajectoryOption(String name, Pose2d traj){
        for(SendableChooser<Pose2d> action : trajectoryActions)
            action.addOption(name, traj);
    }

    private void addDefaultTrajectoryOption(String name, Pose2d traj){
        for(SendableChooser<Pose2d> action : trajectoryActions)
            action.setDefaultOption(name, traj);
    }

    private void addToSmartDashboard(){
        for(int i = 0; i < trajectoryActions.size(); i++)
            SmartDashboard.putData("TrajectoryAction "+i, trajectoryActions.get(i));
    }
}
