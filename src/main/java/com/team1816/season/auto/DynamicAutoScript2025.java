package com.team1816.season.auto;

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
import java.util.HashMap;

public class DynamicAutoScript2025 {
    private SendableChooser<Pose2d> startingPositionChooser = new SendableChooser<>();
    private Pose2d startPos = new Pose2d(new Translation2d(7.55,7.31), Rotation2d.fromDegrees(180));
    private Pose2d currentStartPos = startPos;
    private ArrayList<SendableChooser<Pose2d>> trajectoryActionChoosers = new ArrayList<>();
    private ArrayList<Pose2d> currentTrajectoryActionChoices = new ArrayList<>();
    private ArrayList<TrajectoryAction> autoTrajectoryActions = new ArrayList<>();
    private HashMap<String, Pose2d> allDynamicPoints = new HashMap<>();

    private Robot robot = Injector.get(Robot.class);

    public DynamicAutoScript2025(int numOfTrajectoryActions){
        addDefaultStartPosOption("Start Top", new Pose2d(new Translation2d(7.55,7.31), Rotation2d.fromDegrees(180)));
        addStartPosOption("Start Mid", new Pose2d(new Translation2d(7.55,4.2), Rotation2d.fromDegrees(180)));
        addStartPosOption("Start Bot", new Pose2d(new Translation2d(7.55,.74), Rotation2d.fromDegrees(180)));

        for(int i = 0; i < numOfTrajectoryActions; i++)
            trajectoryActionChoosers.add(new SendableChooser<>());

        addDefaultTrajectoryOption("Hex 1 Blue", new Pose2d(new Translation2d(5.203654,5.259367), Rotation2d.fromDegrees(240)));
        addTrajectoryOption("Hex 2 Blue", new Pose2d(new Translation2d(5.857232,4.049038), Rotation2d.fromDegrees(180)));
        addTrajectoryOption("Hex 3 Blue", new Pose2d(new Translation2d(5.155241,2.814502), Rotation2d.fromDegrees(120)));
        addTrajectoryOption("Hex 4 Blue", new Pose2d(new Translation2d(3.751259,2.838709), Rotation2d.fromDegrees(60)));
        addTrajectoryOption("Hex 5 Blue", new Pose2d(new Translation2d(3.049268, 4.097451), Rotation2d.fromDegrees(0)));
        addTrajectoryOption("Hex 6 Blue", new Pose2d(new Translation2d(3.799672, 5.210954), Rotation2d.fromDegrees(300)));
        addTrajectoryOption("HP Top Blue", new Pose2d(new Translation2d(1.185360, 6.857003), Rotation2d.fromDegrees(130)));
        addTrajectoryOption("HP Bot Blue", new Pose2d(new Translation2d(1.233773, 1.265280), Rotation2d.fromDegrees(230)));
        addTrajectoryOption("Hex 1 Red", new Pose2d(new Translation2d(13.675961, 5.307781), Rotation2d.fromDegrees(240)));
        addTrajectoryOption("Hex 2 Red", new Pose2d(new Translation2d(14.426365, 4.097451), Rotation2d.fromDegrees(180)));
        addTrajectoryOption("Hex 3 Red", new Pose2d(new Translation2d(13.748580, 2.814502), Rotation2d.fromDegrees(120)));
        addTrajectoryOption("Hex 4 Red", new Pose2d(new Translation2d(12.368805, 2.838709), Rotation2d.fromDegrees(60)));
        addTrajectoryOption("Hex 5 Red", new Pose2d(new Translation2d(11.618401, 4.049038), Rotation2d.fromDegrees(0)));
        addTrajectoryOption("Hex 6 Red", new Pose2d(new Translation2d(12.344598, 5.307781), Rotation2d.fromDegrees(300)));
        addTrajectoryOption("HP Top Red", new Pose2d(new Translation2d(16.266066, 6.881209), Rotation2d.fromDegrees(50)));
        addTrajectoryOption("HP Bot Red", new Pose2d(new Translation2d(16.241859, 1.192661), Rotation2d.fromDegrees(310)));



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

    public HashMap<String, Pose2d> getAllDynamicPoints(){
        return allDynamicPoints;
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

    private void addStartPosOption(String name, Pose2d traj){
        allDynamicPoints.put(name, traj);
        startingPositionChooser.addOption(name, traj);
    }

    private void addDefaultStartPosOption(String name, Pose2d traj){
        allDynamicPoints.put(name, traj);
        startingPositionChooser.setDefaultOption(name, traj);
    }

    private void addTrajectoryOption(String name, Pose2d traj){
        allDynamicPoints.put(name, traj);
        for(SendableChooser<Pose2d> action : trajectoryActionChoosers)
            action.addOption(name, traj);
    }

    private void addDefaultTrajectoryOption(String name, Pose2d traj){
        allDynamicPoints.put(name, traj);
        for(SendableChooser<Pose2d> action : trajectoryActionChoosers)
            action.setDefaultOption(name, traj);
    }

    private void addToSmartDashboard(){
        SmartDashboard.putData("DStartingPosition", startingPositionChooser);

        for(int i = 0; i < trajectoryActionChoosers.size(); i++)
            SmartDashboard.putData("DTrajectoryAction "+i, trajectoryActionChoosers.get(i));
    }
}
