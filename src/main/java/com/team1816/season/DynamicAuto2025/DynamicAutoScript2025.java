package com.team1816.season.DynamicAuto2025;

import com.team1816.core.Robot;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.TrajectoryAction;
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
    private SendableChooser<Pose2d> startingPositionChooser = new SendableChooser<>();
    private Pose2d startPos = new Pose2d(new Translation2d(2,1), new Rotation2d());
    private Pose2d currentStartPos = startPos;
    private ArrayList<SendableChooser<Pose2d>> trajectoryActionChoosers = new ArrayList<>();
    private ArrayList<Pose2d> currentTrajectoryActionChoices = new ArrayList<>();
    private ArrayList<TrajectoryAction> autoTrajectoryActions = new ArrayList<>();

    private Robot robot = Injector.get(Robot.class);

    public DynamicAutoScript2025(int numOfTrajectoryActions){
        startingPositionChooser.setDefaultOption("Position 1", new Pose2d(new Translation2d(8,3), new Rotation2d()));
        startingPositionChooser.setDefaultOption("Position 2", new Pose2d(new Translation2d(10,4), new Rotation2d()));

        for(int i = 0; i < numOfTrajectoryActions; i++)
            trajectoryActionChoosers.add(new SendableChooser<>());

        addDefaultTrajectoryOption("somewhere", new Pose2d(new Translation2d(3,1), new Rotation2d()));
        addTrajectoryOption("elsewhere", new Pose2d(new Translation2d(5,1), new Rotation2d()));
        addTrajectoryOption("narnia", new Pose2d(new Translation2d(2,5), new Rotation2d()));
        addTrajectoryOption("space", new Pose2d(new Translation2d(14,4), new Rotation2d()));

        for(SendableChooser<Pose2d> chooser : trajectoryActionChoosers)
            currentTrajectoryActionChoices.add(chooser.getSelected());

        addToSmartDashboard();
    }

    public void updateSendableChoosers(){
        boolean somethingChanged = false;

        Pose2d selected = startingPositionChooser.getSelected();
        if(!startPos.equals(selected)) {
            somethingChanged = true;
            startPos = startingPositionChooser.getSelected();
        }

        ArrayList<TrajectoryAction> newAutoTrajectoryActions = new ArrayList<>();
        currentStartPos = startPos;
        for(int i = 0; i < trajectoryActionChoosers.size(); i++) {
            SendableChooser<Pose2d> trajectoryActionChooser = trajectoryActionChoosers.get(i);

            Pose2d selected2 = trajectoryActionChooser.getSelected();
            if(!currentTrajectoryActionChoices.get(i).equals(selected2)) {
                currentTrajectoryActionChoices.set(i, trajectoryActionChooser.getSelected());
                somethingChanged = true;
            }

            Trajectory newTraj = AutopathAlgorithm.calculateAutopath(currentStartPos, trajectoryActionChooser.getSelected());
            TrajectoryAction newTrajAction;
            assert newTraj != null;
            if(newTraj.getStates().isEmpty())
                newTrajAction = null;
            else
                newTrajAction = new TrajectoryAction(newTraj, List.of(Rotation2d.fromDegrees(0)));
            newAutoTrajectoryActions.add(newTrajAction);

            currentStartPos = trajectoryActionChooser.getSelected();
        }

        autoTrajectoryActions = newAutoTrajectoryActions;

        if(somethingChanged && robot.isDisabled())
            RobotState.dynamicAutoChanged = true;
    }

    public ArrayList<TrajectoryAction> getAutoTrajectoryActions(){
//        for(TrajectoryAction action : autoTrajectoryActions){
//            System.out.println(action.getTrajectory());
//        }
        return autoTrajectoryActions;
    }

    public ArrayList<TrajectoryAction> getAutoTrajectoryActionsIgnoreEmpty(){
        ArrayList<TrajectoryAction> culledAutoTrajectoryActions = new ArrayList<>();

        for(TrajectoryAction action : autoTrajectoryActions)
            if(action != null && !action.getTrajectory().getStates().isEmpty())
                culledAutoTrajectoryActions.add(action);

        return culledAutoTrajectoryActions;
    }

    public ArrayList<TrajectoryAction> getAutoTrajectoryActionsIgnoreEmptyOriented(Rotation2d targetRotation){
        ArrayList<TrajectoryAction> culledAutoTrajectoryActions = new ArrayList<>();

        for(TrajectoryAction action : autoTrajectoryActions)
            if(action != null && !action.getTrajectory().getStates().isEmpty())
                culledAutoTrajectoryActions.add(new TrajectoryAction(action.trajectory, List.of(targetRotation)));

        return culledAutoTrajectoryActions;
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    private void addTrajectoryOption(String name, Pose2d traj){
        for(SendableChooser<Pose2d> action : trajectoryActionChoosers)
            action.addOption(name, traj);
    }

    private void addDefaultTrajectoryOption(String name, Pose2d traj){
        for(SendableChooser<Pose2d> action : trajectoryActionChoosers)
            action.setDefaultOption(name, traj);
    }

    private void addToSmartDashboard(){
        SmartDashboard.putData("DStartingPosition", startingPositionChooser);

        for(int i = 0; i < trajectoryActionChoosers.size(); i++)
            SmartDashboard.putData("DTrajectoryAction "+i, trajectoryActionChoosers.get(i));
    }
}
