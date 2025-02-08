package com.team1816.lib.autopath;

import com.team1816.lib.DriveFactory;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.SeriesAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.auto.actions.WaitAction;
import com.team1816.lib.auto.paths.PathUtil;
import com.team1816.lib.subsystems.drive.EnhancedSwerveDrive;
import com.team1816.lib.util.logUtil.GreenLogger;
import com.team1816.core.auto.AutoModeManager;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import jakarta.inject.Singleton;
import org.apache.commons.math3.Field;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import java.lang.reflect.Array;
import java.sql.Time;
import java.util.ArrayList;
import java.util.List;

@Singleton
public class Autopath {

    public static RobotState robotState;

    private final long looperDtInMS = (long) (Constants.kLooperDt * 1000);

    private Pose2d autopathTargetPosition = new Pose2d(0,0,new Rotation2d(0));

    private static FieldMap stableFieldMap = new FieldMap(/*1755/2, 805/2*/1, 1);

    public static UpdatableAndExpandableFieldMap fieldMap;
    public static UpdatableAndExpandableFieldMap fieldMapExpanded;

    private Pose2d autopathStartPosition = null;

    private TrajectoryAction autopathTrajectoryAction;

    public static int autopathTrajectoryPathCheckPrecisionInTimesPerSecond = 50;

    public static final double mapResolution1DPerMeter = 50;

    /**
     * State: if path needs to be stopped
     */
    private boolean needsStop;

    /**
     * Initializes Autopath
     */
    public Autopath() {
        robotState = Injector.get(RobotState.class);

//        Noah is the best
//        stableFieldMap.drawPolygon(new int[]{368/2, 449/2, 530/2, 530/2, 449/2, 368/2}, new int[]{353/2, 310/2, 353/2, 453/2, 495/2, 453/2}, true);
//        stableFieldMap.drawPolygon(new int[]{1225/2, 1306/2, 1387/2, 1387/2, 1306/2, 1225/2}, new int[]{353/2, 310/2, 353/2, 453/2, 495/2, 453/2}, true);
//        stableFieldMap.drawPolygon(new int[]{850/2, 850/2, 910/2, 910/2}, new int[]{420/2, 390/2, 390/2, 420/2}, true);
//        stableFieldMap.drawPolygon(new int[]{0/2, 170/2, 0/2}, new int[]{0/2, 0/2, 150/2}, true);
//        stableFieldMap.drawPolygon(new int[]{0/2, 170/2, 0/2}, new int[]{805/2, 805/2, 655/2}, true);
//        stableFieldMap.drawPolygon(new int[]{1755/2, 1585/2, 1755/2}, new int[]{0/2, 0/2, 150/2}, true);
//        stableFieldMap.drawPolygon(new int[]{1755/2, 1585/2, 1755/2}, new int[]{805/2, 805/2, 655/2}, true);

        fieldMap = new UpdatableAndExpandableFieldMap(stableFieldMap.getMapX(), stableFieldMap.getMapY(), stableFieldMap, new FieldMap(stableFieldMap.getMapX(), stableFieldMap.getMapY()), 59.26969039916799/2);
        fieldMapExpanded = new UpdatableAndExpandableFieldMap(stableFieldMap.getMapX(), stableFieldMap.getMapY(), stableFieldMap, new FieldMap(stableFieldMap.getMapX(), stableFieldMap.getMapY()), 65/2);
    }

    /**
     * Tests a trajectory against the fieldMap to see whether a robot of (whatever) width can path successfully
     *
     * @param trajectory
     * @return
     */
    public static boolean testTrajectory(Trajectory trajectory){
//        if(trajectory == null)
//            return false;
//
//        Pose2d prevState = trajectory.sample(0).poseMeters;
//
//        for(int t = 1; t*.02 < trajectory.getTotalTimeSeconds() + 1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond; t++){
//            Pose2d currentState = trajectory.sample(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
//            if(fieldMap.getCurrentMap().checkPixelHasObjectOrOffMap((int)(currentState.getX()*mapResolution1DPerMeter), (int)(currentState.getY()*mapResolution1DPerMeter)))
//                return false;
//
//            prevState = currentState;
//        }

        return true;
    }

    public static TimestampTranslation2d returnCollisionStart(Trajectory trajectory){
//        Pose2d prevState = trajectory.sample(0).poseMeters;
//
//        for(int t = 1; t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond < trajectory.getTotalTimeSeconds() + 1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond; t++){
//            Pose2d currentState = trajectory.sample(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
//            Translation2d result = Bresenham.lineReturnCollision(fieldMap.getCurrentMap(), (int)(prevState.getX()*mapResolution1DPerMeter), (int)(prevState.getY()*mapResolution1DPerMeter), (int)(currentState.getX()*mapResolution1DPerMeter), (int)(currentState.getY()*mapResolution1DPerMeter));
//
//            if(result != null)
//                return new TimestampTranslation2d(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond, result);
//
//            prevState = currentState;
//        }
        return null;
    }

    public static TimestampTranslation2d returnCollisionEnd(Trajectory trajectory, TimestampTranslation2d timestampTranslation2d){

//        Pose2d prevState = trajectory.sample(timestampTranslation2d.getTimestamp()).poseMeters;
//
//        for(int t = (int)(timestampTranslation2d.getTimestamp()*autopathTrajectoryPathCheckPrecisionInTimesPerSecond) + 1; t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond < trajectory.getTotalTimeSeconds() + 1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond; t++){
//            Pose2d currentState = trajectory.sample(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
//
//
//            int[] result =
//                    Bresenham.lineReturnCollisionInverted(
//                            fieldMap.getCurrentMap(),
//                            (int)(prevState.getX()*mapResolution1DPerMeter),
//                            (int)(prevState.getY()*mapResolution1DPerMeter),
//                            (int)(currentState.getX()*mapResolution1DPerMeter),
//                            (int)(currentState.getY()*mapResolution1DPerMeter),
//                            true
//                    );
//
//            if(result != null)
//                return new TimestampTranslation2d(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond, new Translation2d(result[0], result[1]));
//
//            prevState = currentState;
//        }

        return timestampTranslation2d;
    }

    public static TimestampTranslation2d returnCollisionStartLast(Trajectory trajectory){
//        Pose2d prevState = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
//
//        for(int t = (int)(trajectory.getTotalTimeSeconds()*autopathTrajectoryPathCheckPrecisionInTimesPerSecond) - 1; t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond > 0; t--){
//            Pose2d currentState = trajectory.sample(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
//            Translation2d result = Bresenham.lineReturnCollision(fieldMap.getCurrentMap(), (int)(prevState.getX()*mapResolution1DPerMeter), (int)(prevState.getY()*mapResolution1DPerMeter), (int)(currentState.getX()*mapResolution1DPerMeter), (int)(currentState.getY()*mapResolution1DPerMeter));
//
//            if(result != null)
//                return new TimestampTranslation2d(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond, result);
//
//            prevState = currentState;
//        }
        return null;
    }

    public static TimestampTranslation2d returnCollisionEndLast(Trajectory trajectory, TimestampTranslation2d timestampTranslation2d){
//        Pose2d prevState = trajectory.sample(timestampTranslation2d.getTimestamp()).poseMeters;
//
//        for(int t = (int)(timestampTranslation2d.getTimestamp()*autopathTrajectoryPathCheckPrecisionInTimesPerSecond) - 1; t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond > 0; t--){
//            Pose2d currentState = trajectory.sample(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
//
////            System.out.println("Testing line: "+prevState+" to: "+currentState);
//
//            int[] result =
//                    Bresenham.lineReturnCollisionInverted(
//                            fieldMap.getCurrentMap(),
//                            (int)(prevState.getX()*mapResolution1DPerMeter),
//                            (int)(prevState.getY()*mapResolution1DPerMeter),
//                            (int)(currentState.getX()*mapResolution1DPerMeter),
//                            (int)(currentState.getY()*mapResolution1DPerMeter),
//                            true
//                    );
//
//            if(result != null)
//                return new TimestampTranslation2d(t*1./autopathTrajectoryPathCheckPrecisionInTimesPerSecond, new Translation2d(result[0], result[1]));
//
//            prevState = currentState;
//        }

        return timestampTranslation2d;
    }

    /**
     * Runs the Autopath routine actions
     *
     * @see #routine()
     */
    public void run(Pose2d autopathTargetPosition) {
        this.autopathTargetPosition = autopathTargetPosition;

        System.out.println("You told me to do something!");

//        start();

        try {
            routine();
        } catch (Exception e){
            GreenLogger.log("Autopathing ended early");
        }

        done();
    }

    /**
     * Runs the Autopath routine actions
     *
     * @see #routine()
     */
    private void run(Translation2d autopathTargetPosition) {
        run(new Pose2d(autopathTargetPosition, robotState.fieldToVehicle.getRotation()));
    }

    /**
     * Starts the Autopath and relevant actions
     */
    public void start(Pose2d autopathTargetPosition) {
        if(robotState.autopathing && System.nanoTime()/1000000 - robotState.autopathBeforeTime < robotState.autopathPathCancelBufferMilli)
            return;

        this.autopathTargetPosition = autopathTargetPosition;

        robotState.autopathing = true;

        autopathStartPosition = robotState.fieldToVehicle;

        GreenLogger.log("Starting Autopath");
        needsStop = false;

        Trajectory autopathTrajectory;

        robotState.autopathBeforeTime = (double) System.nanoTime() /1000000;
        double beforeTime = robotState.autopathBeforeTime;

        autopathTrajectory = AutopathAlgorithm.calculateAutopath(autopathTargetPosition);

        if(autopathTrajectory == null){
            robotState.autopathing = false;
            return;
        }

        System.out.println("Time taken "+(System.nanoTime()-beforeTime)/1000000000);

        ArrayList<Rotation2d> autopathHeadings = new ArrayList<>();
        autopathHeadings.add(autopathTargetPosition.getRotation());
//        double autopathTrajectoryTime = autopathTrajectory.getTotalTimeSeconds();
//        for(int i = 0; i < autopathTrajectory.getStates().size(); i++){
//            autopathHeadings.add(Rotation2d.fromDegrees(
//                    autopathStartPosition.getRotation().getDegrees() * (autopathTrajectoryTime - autopathTrajectory.getStates().get(i).timeSeconds) / autopathTrajectoryTime +
//                            autopathTargetPosition.getRotation().getDegrees() * autopathTrajectory.getStates().get(i).timeSeconds / autopathTrajectoryTime
//            ));
//        }

//        System.out.println(autopathHeadings.get(0));
//        System.out.println(autopathHeadings.get(autopathHeadings.size()-1));
//        System.out.println(autopathHeadings);

        //Here's where your trajectory gets checked against the field
//        System.out.println("And survey says: "+testTrajectory(autopathTrajectory));

        autopathTrajectoryAction = new TrajectoryAction(autopathTrajectory, autopathHeadings);

        autopathTrajectoryAction.start();
    }

    /**
     * Called every loop
     */
    public void routine() throws AutoModeEndedException {
//        GreenLogger.log("Autopathing Running");

        // Run actions here:
        // e.g. runAction(new SeriesAction(new WaitAction(0.5), ...))

        // Run action, stop action on interrupt or done
        if (!autopathTrajectoryAction.isFinished()) {
            if (needsStop) {
                throw new AutoModeEndedException();
            }

            autopathTrajectoryAction.update();

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        else{
            autopathTrajectoryAction.done();
            done();
        }
    }

    /**
     * Standard cleanup end-procedure
     */
    protected void done() {
        robotState.autopathing = false;

        System.out.println("Started at "+autopathStartPosition);
        System.out.println("Hopefully ended at "+autopathTargetPosition);
        System.out.println("And it thinks it's at "+robotState.fieldToVehicle);

        GreenLogger.log("Autopath Done");
    }

    /**
     * Stops the auto path
     */
    public void stop() {
        autopathTrajectoryAction.done();
        robotState.autopathing = false;
    }

    /**
     * Runs a given action, typically placed in routine()
     *
     * @param action
     * @throws AutoModeEndedException
     * @see AutoAction
     */
    protected void runAction(AutoAction action) throws AutoModeEndedException {
        action.start();

        // Run action, stop action on interrupt or done
        while (!action.isFinished()) {
            if (needsStop) {
                throw new AutoModeEndedException();
            }

            action.update();

//            try {
//                Thread.sleep(looperDtInMS);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
        }

        action.done();
    }

    public static class TimestampTranslation2d{
        private double timestamp;
        private Translation2d translation2d;

        public TimestampTranslation2d(double timestamp, Translation2d translation2d){
            this.timestamp = timestamp;
            this.translation2d = translation2d;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public void setTimestamp(double timestamp) {
            this.timestamp = timestamp;
        }

        public Translation2d getTranslation2d() {
            return translation2d;
        }

        public void setTranslation2d(Translation2d translation2d) {
            this.translation2d = translation2d;
        }
    }
}