package com.team1816.season.auto;

import com.team1816.core.Robot;
import com.team1816.core.auto.AutoModeManager;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.autopath.AutopathAlgorithm;
import com.team1816.season.subsystems.CoralArm;
import com.team1816.season.subsystems.Elevator;
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
    private Color color = Color.BLUE;

    private HashMap<String, Pose2d> allDynamicPoints = new HashMap<>();

    private SendableChooser<Pose2d> startingPositionChooser = new SendableChooser<>();
    private Pose2d startPos;

    private ArrayList<SendableChooser<Pose2d>> trajectoryActionChoosers = new ArrayList<>();
    private ArrayList<Pose2d> currentTrajectoryActionChoices = new ArrayList<>();
    private ArrayList<TrajectoryAction> autoTrajectoryActions = new ArrayList<>();

    private ArrayList<SendableChooser<REEF_LEVEL>> coralPlacementChoosers = new ArrayList<>();
    private ArrayList<REEF_LEVEL> currentCoralPlacementChoices = new ArrayList<>();

    private Robot robot;
    private RobotState robotState;

    public DynamicAutoScript2025(int numOfTrajectoryActions, int numOfCoralPlacementActions){
        robot = Injector.get(Robot.class);
        robotState = Injector.get(RobotState.class);

        addDefaultStartPosOption("Start Top", new Pose2d(new Translation2d(7.55,7.31), Rotation2d.fromDegrees(180)));
        addStartPosOption("Start Mid", new Pose2d(new Translation2d(7.55,4.2), Rotation2d.fromDegrees(180)));
        addStartPosOption("Start Bot", new Pose2d(new Translation2d(7.55,.74), Rotation2d.fromDegrees(180)));

        startPos = startingPositionChooser.getSelected();

        for(int i = 0; i < numOfTrajectoryActions; i++)
            trajectoryActionChoosers.add(new SendableChooser<>());

        addDefaultTrajectoryOption("Hex 1", new Pose2d(new Translation2d(5.142950,5.338677), Rotation2d.fromDegrees(240)));
        addTrajectoryOption("Hex 2", new Pose2d(new Translation2d(6.219579,4.049038), Rotation2d.fromDegrees(180)));
        addTrajectoryOption("Hex 3", new Pose2d(new Translation2d(5.155241,2.586418), Rotation2d.fromDegrees(120)));
        addTrajectoryOption("Hex 4", new Pose2d(new Translation2d(3.751259,2.586418), Rotation2d.fromDegrees(60)));
        addTrajectoryOption("Hex 5", new Pose2d(new Translation2d(3.049268, 3.020218), Rotation2d.fromDegrees(0)));
        addTrajectoryOption("Hex 6", new Pose2d(new Translation2d(3.799672, 5.406116), Rotation2d.fromDegrees(300)));
        addTrajectoryOption("Feeder Top", new Pose2d(new Translation2d(1.185360, 6.571952), Rotation2d.fromDegrees(130)));
        addTrajectoryOption("Feeder Bot", new Pose2d(new Translation2d(1.185360, 1.565280), Rotation2d.fromDegrees(230)));

        for(SendableChooser<Pose2d> chooser : trajectoryActionChoosers)
            currentTrajectoryActionChoices.add(chooser.getSelected());

        for(int i = 0; i < numOfCoralPlacementActions; i++)
            coralPlacementChoosers.add(new SendableChooser<>());

        addDefaultCoralPlacementOption("Reef L1", REEF_LEVEL.L1);
        addCoralPlacementOption("Reef L2", REEF_LEVEL.L2);
        addCoralPlacementOption("Reef L3", REEF_LEVEL.L3);
        addCoralPlacementOption("Reef L4", REEF_LEVEL.L4);

        for(SendableChooser<REEF_LEVEL> chooser: coralPlacementChoosers)
            currentCoralPlacementChoices.add(chooser.getSelected());

        addToSmartDashboard();
    }

    public void updateSendableChoosers(){
        boolean somethingChanged = false;
        if(color != robotState.allianceColor)
            somethingChanged = true;
        color = robotState.allianceColor;

        Pose2d selected = startingPositionChooser.getSelected();
        if(!startPos.equals(selected)) {
            somethingChanged = true;
            startPos = selected;
        }

        ArrayList<TrajectoryAction> newAutoTrajectoryActions = new ArrayList<>();
        Pose2d currentStartPos = startPos;
        for (int i = 0; i < trajectoryActionChoosers.size(); i++) {
            SendableChooser<Pose2d> trajectoryActionChooser = trajectoryActionChoosers.get(i);

            Pose2d selected2 = trajectoryActionChooser.getSelected();
            if (!currentTrajectoryActionChoices.get(i).equals(selected2)) {
                currentTrajectoryActionChoices.set(i, selected2);
                somethingChanged = true;
            }

            if(color == Color.BLUE) {
                Trajectory newTraj = AutopathAlgorithm.calculateAutopath(currentStartPos, trajectoryActionChooser.getSelected());
                TrajectoryAction newTrajAction = null;
                if (newTraj != null) {
                    if (!newTraj.getStates().isEmpty()) {
                        newTrajAction = new TrajectoryAction(newTraj, Collections.nCopies(newTraj.getStates().size(), trajectoryActionChooser.getSelected().getRotation()));
                    }
                    newAutoTrajectoryActions.add(newTrajAction);
                }
            } else if(color == Color.RED) {
                Pose2d redCurrentStartPos = new Pose2d(2 * Constants.fieldCenterX - currentStartPos.getX(), 2 * Constants.fieldCenterY - currentStartPos.getY(), Rotation2d.fromDegrees(180 + currentStartPos.getRotation().getDegrees()));
                Pose2d targetPos = new Pose2d(2 * Constants.fieldCenterX - trajectoryActionChooser.getSelected().getX(), 2 * Constants.fieldCenterY - trajectoryActionChooser.getSelected().getY(), Rotation2d.fromDegrees(180 + trajectoryActionChooser.getSelected().getRotation().getDegrees()));

                Trajectory newTraj = AutopathAlgorithm.calculateAutopath(redCurrentStartPos, targetPos);
                TrajectoryAction newTrajAction = null;
                if (newTraj != null) {
                    if (!newTraj.getStates().isEmpty()) {
                        newTrajAction = new TrajectoryAction(newTraj, Collections.nCopies(newTraj.getStates().size(), targetPos.getRotation()));
                    }
                    newAutoTrajectoryActions.add(newTrajAction);
                }
            } else
                System.out.println("idk how the fuck you got me to print");

            currentStartPos = trajectoryActionChooser.getSelected();
        }
        autoTrajectoryActions = newAutoTrajectoryActions;

        ArrayList<REEF_LEVEL> newCurrentCoralPlacementChoices = new ArrayList<>();
        for (int i = 0; i < coralPlacementChoosers.size(); i++){
            newCurrentCoralPlacementChoices.add(coralPlacementChoosers.get(i).getSelected());
        }
        currentCoralPlacementChoices = newCurrentCoralPlacementChoices;

        if(somethingChanged && robot.isDisabled()) {
            robotState.dynamicAutoChanged = true;
            Injector.get(AutoModeManager.class).updateAutoMode();
        }
    }

    public HashMap<String, Pose2d> getAllDynamicPoints(){
        return allDynamicPoints;
    }

    public Pose2d getStartPos(){
        if(color == Color.BLUE)
            return startPos;
        else
            return new Pose2d(2 * Constants.fieldCenterX - startPos.getX(), 2 * Constants.fieldCenterY - startPos.getY(), Rotation2d.fromDegrees(180 + startPos.getRotation().getDegrees()));
    }

    public ArrayList<TrajectoryAction> getAutoTrajectoryActionsIgnoreEmpty(){
        ArrayList<TrajectoryAction> culledAutoTrajectoryActions = new ArrayList<>();

        for(TrajectoryAction action : autoTrajectoryActions)
            if(action != null && !action.getTrajectory().getStates().isEmpty())
                culledAutoTrajectoryActions.add(action);

        return culledAutoTrajectoryActions;
    }

    public ArrayList<REEF_LEVEL> getCoralPlacements(){
        return currentCoralPlacementChoices;
    }

    private void addStartPosOption(String name, Pose2d pos){
        allDynamicPoints.put(name, pos);
        startingPositionChooser.addOption(name, pos);
    }

    private void addDefaultStartPosOption(String name, Pose2d pos){
        allDynamicPoints.put(name, pos);
        startingPositionChooser.setDefaultOption(name, pos);
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

    private void addCoralPlacementOption(String name, REEF_LEVEL state){
        for(SendableChooser<REEF_LEVEL> chooser : coralPlacementChoosers)
            chooser.addOption(name, state);
    }

    private void addDefaultCoralPlacementOption(String name, REEF_LEVEL state){
        for(SendableChooser<REEF_LEVEL> chooser : coralPlacementChoosers)
            chooser.setDefaultOption(name, state);
    }

    private void addToSmartDashboard(){
        SmartDashboard.putData("DStartingPosition", startingPositionChooser);

        for(int i = 0; i < trajectoryActionChoosers.size(); i++)
            SmartDashboard.putData("DTrajectoryAction "+i, trajectoryActionChoosers.get(i));

        for(int i = 0; i < coralPlacementChoosers.size(); i++)
            SmartDashboard.putData("DCoralPlacement "+i, coralPlacementChoosers.get(i));
    }

    public enum REEF_LEVEL{
        L1,

        L2,

        L3,

        L4;

        public Elevator.ELEVATOR_STATE getEquivalentElevatorState() {
            switch (this) {
                case L1 -> {
                    return Elevator.ELEVATOR_STATE.L1;
                }
                case L2 -> {
                    return Elevator.ELEVATOR_STATE.L2;
                }
                case L3 -> {
                    return Elevator.ELEVATOR_STATE.L3;
                }
                case L4 -> {
                    return Elevator.ELEVATOR_STATE.L4;
                }
            }

            return null;
        }

        public CoralArm.PIVOT_STATE getEquivalentCoralArmPivotState() {
            switch (this) {
                case L1 -> {
                    return CoralArm.PIVOT_STATE.L1;
                }
                case L2 -> {
                    return CoralArm.PIVOT_STATE.L2;
                }
                case L3 -> {
                    return CoralArm.PIVOT_STATE.L3;
                }
                case L4 -> {
                    return CoralArm.PIVOT_STATE.L4;
                }
            }

            return null;
        }
    }
}
