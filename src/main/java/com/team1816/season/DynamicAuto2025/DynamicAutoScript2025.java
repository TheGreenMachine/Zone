package com.team1816.season.DynamicAuto2025;

import com.team1816.core.Robot;
import com.team1816.core.auto.AutoModeManager;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.autopath.Autopath;
import com.team1816.lib.autopath.AutopathAlgorithm;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class DynamicAutoScript2025 {
    private SendableChooser<Pose2d> startingPositionChooser = new SendableChooser<>();
    private Pose2d startPos = new Pose2d(new Translation2d(2,1), new Rotation2d());
    private Pose2d currentStartPos = startPos;
    private ArrayList<SendableChooser<Autopath.TransformedAutopathTranslations>> trajectoryActionChoosers = new ArrayList<>();
    private ArrayList<Autopath.TransformedAutopathTranslations> currentTrajectoryActionChoices = new ArrayList<>();
    private ArrayList<TrajectoryAction> autoTrajectoryActions = new ArrayList<>();

    private Robot robot = Injector.get(Robot.class);

    public DynamicAutoScript2025(int numOfTrajectoryActions){
        startingPositionChooser.setDefaultOption("Position 1", new Pose2d(new Translation2d(8,3), Rotation2d.fromDegrees(45)));
        startingPositionChooser.setDefaultOption("Position 2", new Pose2d(new Translation2d(10,4), Rotation2d.fromDegrees(-45)));

        for(int i = 0; i < numOfTrajectoryActions; i++)
            trajectoryActionChoosers.add(new SendableChooser<>());

        addDefaultTrajectoryOption("Hex 1 Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(5.203654,5.259367), Rotation2d.fromDegrees(240))));
        addTrajectoryOption("Hex 2 Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(5.857232,4.049038), Rotation2d.fromDegrees(180))));
        addTrajectoryOption("Hex 3 Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(5.155241,2.814502), Rotation2d.fromDegrees(120))));
        addTrajectoryOption("Hex 4 Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(3.751259,2.838709), Rotation2d.fromDegrees(60))));
        addTrajectoryOption("Hex 5 Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(3.049268, 4.097451), Rotation2d.fromDegrees(0))));
        addTrajectoryOption("Hex 6 Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(3.799672, 5.210954), Rotation2d.fromDegrees(320))));
        addTrajectoryOption("HP Top Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(1.185360, 6.857003), Rotation2d.fromDegrees(130))));
        addTrajectoryOption("HP Bot Blue", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(1.233773, 1.265280), Rotation2d.fromDegrees(230))));
        addTrajectoryOption("Hex 1 Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(13.675961, 5.307781), Rotation2d.fromDegrees(240))));
        addTrajectoryOption("Hex 2 Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(14.426365, 4.097451), Rotation2d.fromDegrees(180))));
        addTrajectoryOption("Hex 3 Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(13.748580, 2.814502), Rotation2d.fromDegrees(120))));
        addTrajectoryOption("Hex 4 Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(12.368805, 2.838709), Rotation2d.fromDegrees(60))));
        addTrajectoryOption("Hex 5 Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(11.618401, 4.049038), Rotation2d.fromDegrees(0))));
        addTrajectoryOption("Hex 6 Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(12.344598, 5.307781), Rotation2d.fromDegrees(320))));
        addTrajectoryOption("HP Top Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(16.266066, 6.881209), Rotation2d.fromDegrees(50))));
        addTrajectoryOption("HP Bot Red", Autopath.getTransformedAutopathTranslations(new Pose2d(new Translation2d(16.241859, 1.192661), Rotation2d.fromDegrees(310))));



        for(SendableChooser<Autopath.TransformedAutopathTranslations> chooser : trajectoryActionChoosers)
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

        ArrayList<TrajectoryAction> newAutoTrajectoryActions = new ArrayList<>(autoTrajectoryActions);
        currentStartPos = startPos;


        for (int i = 0; i < trajectoryActionChoosers.size(); i++) {
            SendableChooser<Autopath.TransformedAutopathTranslations> trajectoryActionChooser = trajectoryActionChoosers.get(i);

            Autopath.TransformedAutopathTranslations selected2 = trajectoryActionChooser.getSelected();
            if (!currentTrajectoryActionChoices.get(i).equals(selected2)) {
                currentTrajectoryActionChoices.set(i, selected2);
                somethingChanged = true;

                //TODO add method to actually use these correctly(maybe now as of commit on 1/18/2025)
                Trajectory newTraj = AutopathAlgorithm.calculateAutopath(currentStartPos, trajectoryActionChooser.getSelected().biggerRadiusPose, true, true);
                TrajectoryAction newTrajAction = null;
                if (newTraj != null) {
                    if (!newTraj.getStates().isEmpty()) {
                        newTrajAction = new TrajectoryAction(newTraj, Collections.nCopies(newTraj.getStates().size(), trajectoryActionChooser.getSelected().getRotation()));
                    }
                    newAutoTrajectoryActions.set(i, newTrajAction);
                }

                currentStartPos = trajectoryActionChooser.getSelected().biggerRadiusPose;
            }
        }

        if(somethingChanged && robot.isDisabled()) {
            autoTrajectoryActions = newAutoTrajectoryActions;
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

    private void addTrajectoryOption(String name, Autopath.TransformedAutopathTranslations traj){
        for(SendableChooser<Autopath.TransformedAutopathTranslations> action : trajectoryActionChoosers)
            action.addOption(name, traj);
    }

    private void addDefaultTrajectoryOption(String name, Autopath.TransformedAutopathTranslations traj){
        for(SendableChooser<Autopath.TransformedAutopathTranslations> action : trajectoryActionChoosers)
            action.setDefaultOption(name, traj);
    }

    private void addToSmartDashboard(){
        SmartDashboard.putData("DStartingPosition", startingPositionChooser);

        for(int i = 0; i < trajectoryActionChoosers.size(); i++)
            SmartDashboard.putData("DTrajectoryAction "+i, trajectoryActionChoosers.get(i));
    }
}
