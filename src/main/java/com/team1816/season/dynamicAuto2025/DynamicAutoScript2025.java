package com.team1816.season.dynamicAuto2025;

import com.team1816.core.Robot;
import com.team1816.core.auto.AutoModeManager;
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
import java.util.Collections;

public class DynamicAutoScript2025 {
    private SendableChooser<Pose2d> startingPositionChooser = new SendableChooser<>();
    private Pose2d startPos = new Pose2d(new Translation2d(2,1), new Rotation2d());
    private Pose2d currentStartPos = startPos;
    private ArrayList<SendableChooser<Pose2d>> trajectoryActionChoosers = new ArrayList<>();
    private ArrayList<Pose2d> currentTrajectoryActionChoices = new ArrayList<>();
    private ArrayList<TrajectoryAction> autoTrajectoryActions = new ArrayList<>();

    private Robot robot = Injector.get(Robot.class);

    public DynamicAutoScript2025(int numOfTrajectoryActions){
        startingPositionChooser.setDefaultOption("Position 1", new Pose2d(new Translation2d(8,3), Rotation2d.fromDegrees(45)));
        startingPositionChooser.setDefaultOption("Position 2", new Pose2d(new Translation2d(10,4), Rotation2d.fromDegrees(-45)));

        for(int i = 0; i < numOfTrajectoryActions; i++)
            trajectoryActionChoosers.add(new SendableChooser<>());

        addDefaultTrajectoryOption("somewhere", new Pose2d(new Translation2d(3,1), Rotation2d.fromDegrees(90)));
        addTrajectoryOption("elsewhere", new Pose2d(new Translation2d(5,1), Rotation2d.fromDegrees(-90)));
        addTrajectoryOption("narnia", new Pose2d(new Translation2d(2,5), Rotation2d.fromDegrees(180)));
        addTrajectoryOption("space", new Pose2d(new Translation2d(14,4), Rotation2d.fromDegrees(135)));

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
        for (int i = 0; i < trajectoryActionChoosers.size(); i++) {
            SendableChooser<Pose2d> trajectoryActionChooser = trajectoryActionChoosers.get(i);

            Pose2d selected2 = trajectoryActionChooser.getSelected();
            if (!currentTrajectoryActionChoices.get(i).equals(selected2)) {
                currentTrajectoryActionChoices.set(i, selected2);
                somethingChanged = true;
            }

            Trajectory newTraj = AutopathAlgorithm.calculateAutopath(currentStartPos, trajectoryActionChooser.getSelected());
            TrajectoryAction newTrajAction = null;
            if(newTraj != null) {
                if (!newTraj.getStates().isEmpty()){
                    newTrajAction = new TrajectoryAction(newTraj, Collections.nCopies(newTraj.getStates().size(), trajectoryActionChooser.getSelected().getRotation()));
                }
                newAutoTrajectoryActions.add(newTrajAction);
            }

            currentStartPos = trajectoryActionChooser.getSelected();
        }

        autoTrajectoryActions = newAutoTrajectoryActions;

        if(somethingChanged && robot.isDisabled()) {
            RobotState.dynamicAutoChanged = true;
            Injector.get(AutoModeManager.class).updateAutoMode();
        }
    }

    public ArrayList<TrajectoryAction> getAutoTrajectoryActionsIgnoreEmpty(){
        ArrayList<TrajectoryAction> culledAutoTrajectoryActions = new ArrayList<>();

        for(TrajectoryAction action : autoTrajectoryActions)
            if(action != null && !action.getTrajectory().getStates().isEmpty())
                culledAutoTrajectoryActions.add(action);

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
