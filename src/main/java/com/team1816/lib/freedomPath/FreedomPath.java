package com.team1816.lib.freedomPath;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.actions.AutoAction;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.subsystems.drive.Drive;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Singleton
public class FreedomPath {

    public RobotState robotState;

    private final long looperDtInMS = (long) (Constants.kLooperDt * 1000);

    private Pose2d freedomPathTargetPosition = new Pose2d(0,0,new Rotation2d(0));

    private FieldMap stableFieldMap = new FieldMap(1755/2, 805/2/*1, 1*/);

    public UpdatableAndExpandableFieldMap fieldMap;
    public UpdatableAndExpandableFieldMap fieldMapExpanded;

    private Pose2d freedomPathStartPosition = null;

    private TrajectoryAction freedomPathTrajectoryAction;

    public int freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond = 50;

    public final double mapResolution1DPerMeter = 50;

    /**
     * State: if path needs to be stopped
     */
    private boolean needsStop;

    /**
     * Initializes FreedomPath
     */
    @Inject
    public FreedomPath() {
        robotState = Injector.get(RobotState.class);

//        Noah is the best
        stableFieldMap.drawPolygon(new int[]{368/2, 449/2, 530/2, 530/2, 449/2, 368/2}, new int[]{353/2, 310/2, 353/2, 453/2, 495/2, 453/2}, true);
        stableFieldMap.drawPolygon(new int[]{1225/2, 1306/2, 1387/2, 1387/2, 1306/2, 1225/2}, new int[]{353/2, 310/2, 353/2, 453/2, 495/2, 453/2}, true);
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
    public boolean testTrajectory(Trajectory trajectory){
        if(trajectory == null)
            return false;

        Pose2d prevState = trajectory.sample(0).poseMeters;

        for(int t = 1; t*.02 < trajectory.getTotalTimeSeconds() + 1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond; t++){
            Pose2d currentState = trajectory.sample(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
            if(fieldMap.getCurrentMap().checkPixelHasObjectOrOffMap((int)(currentState.getX()*mapResolution1DPerMeter), (int)(currentState.getY()*mapResolution1DPerMeter)))
                return false;

            prevState = currentState;
        }

        return true;
    }

    public TimestampTranslation2d returnCollisionStart(Trajectory trajectory){
        Pose2d prevState = trajectory.sample(0).poseMeters;

        for(int t = 1; t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond < trajectory.getTotalTimeSeconds() + 1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond; t++){
            Pose2d currentState = trajectory.sample(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
            Translation2d result = Bresenham.lineReturnCollision(fieldMap.getCurrentMap(), (int)(prevState.getX()*mapResolution1DPerMeter), (int)(prevState.getY()*mapResolution1DPerMeter), (int)(currentState.getX()*mapResolution1DPerMeter), (int)(currentState.getY()*mapResolution1DPerMeter));

            if(result != null)
                return new TimestampTranslation2d(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond, result);

            prevState = currentState;
        }
        return null;
    }

    public TimestampTranslation2d returnCollisionEnd(Trajectory trajectory, TimestampTranslation2d timestampTranslation2d){

        Pose2d prevState = trajectory.sample(timestampTranslation2d.getTimestamp()).poseMeters;

        for(int t = (int)(timestampTranslation2d.getTimestamp()*freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond) + 1; t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond < trajectory.getTotalTimeSeconds() + 1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond; t++){
            Pose2d currentState = trajectory.sample(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;


            int[] result =
                    Bresenham.lineReturnCollisionInverted(
                            fieldMap.getCurrentMap(),
                            (int)(prevState.getX()*mapResolution1DPerMeter),
                            (int)(prevState.getY()*mapResolution1DPerMeter),
                            (int)(currentState.getX()*mapResolution1DPerMeter),
                            (int)(currentState.getY()*mapResolution1DPerMeter),
                            true
                    );

            if(result != null)
                return new TimestampTranslation2d(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond, new Translation2d(result[0], result[1]));

            prevState = currentState;
        }

        return timestampTranslation2d;
    }

    public TimestampTranslation2d returnCollisionStartLast(Trajectory trajectory){
        Pose2d prevState = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;

        for(int t = (int)(trajectory.getTotalTimeSeconds()*freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond) - 1; t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond > 0; t--){
            Pose2d currentState = trajectory.sample(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;
            Translation2d result = Bresenham.lineReturnCollision(fieldMap.getCurrentMap(), (int)(prevState.getX()*mapResolution1DPerMeter), (int)(prevState.getY()*mapResolution1DPerMeter), (int)(currentState.getX()*mapResolution1DPerMeter), (int)(currentState.getY()*mapResolution1DPerMeter));

            if(result != null)
                return new TimestampTranslation2d(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond, result);

            prevState = currentState;
        }
        return null;
    }

    public TimestampTranslation2d returnCollisionEndLast(Trajectory trajectory, TimestampTranslation2d timestampTranslation2d){
//        System.out.println("Testing position: "+timestampTranslation2d.getTranslation2d()+" at time: "+timestampTranslation2d.getTimestamp());

        Pose2d prevState = trajectory.sample(timestampTranslation2d.getTimestamp()).poseMeters;

        for(int t = (int)(timestampTranslation2d.getTimestamp()*freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond) - 1; t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond > 0; t--){
            Pose2d currentState = trajectory.sample(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond).poseMeters;

//            System.out.println("Testing line: "+prevState+" to: "+currentState);

            int[] result =
                    Bresenham.lineReturnCollisionInverted(
                            fieldMap.getCurrentMap(),
                            (int)(prevState.getX()*mapResolution1DPerMeter),
                            (int)(prevState.getY()*mapResolution1DPerMeter),
                            (int)(currentState.getX()*mapResolution1DPerMeter),
                            (int)(currentState.getY()*mapResolution1DPerMeter),
                            true
                    );

//            System.out.println(result);

            if(result != null)
                return new TimestampTranslation2d(t*1./freedomPathTrajectoryPathCheckPrecisionInTimesPerSecond, new Translation2d(result[0], result[1]));

            prevState = currentState;
        }

        return timestampTranslation2d;
    }

    /**
     * Runs the FreedomPath routine actions
     *
     * @see #routine()
     */
    public void run(Pose2d freedomPathTargetPosition) {
        this.freedomPathTargetPosition = freedomPathTargetPosition;

        System.out.println("You told me to do something!");

//        start();

        try {
            routine();
        } catch (Exception e){
            GreenLogger.log("FreedomPathing ended early");
        }

        done();
    }

    /**
     * Runs the FreedomPath routine actions
     *
     * @see #routine()
     */
    private void run(Translation2d autopathTargetPosition) {
        run(new Pose2d(autopathTargetPosition, robotState.fieldToVehicle.getRotation()));
    }

    /**
     * Starts the Autopath and relevant actions
     */
    public void start(Pose2d freedomPathTargetPosition) {
        if(robotState.isFreedomPathing && System.nanoTime()/1000000 - robotState.freedomPathBeforeTime < robotState.freedomPathCancelBufferMilli)
            return;

        this.freedomPathTargetPosition = freedomPathTargetPosition;

        robotState.isFreedomPathing = true;

        freedomPathStartPosition = robotState.fieldToVehicle;

        GreenLogger.log("Starting FreedomPath");
        needsStop = false;

        Trajectory freedomPathTrajectory;

        robotState.freedomPathBeforeTime = (double) System.nanoTime() /1000000;
        double beforeTime = robotState.freedomPathBeforeTime;

        freedomPathTrajectory = calculateFreedomPath(freedomPathTargetPosition);

        if(freedomPathTrajectory == null){
            robotState.isFreedomPathing = false;
            return;
        }

        System.out.println("Time taken "+(System.nanoTime()-beforeTime)/1000000000);

        ArrayList<Rotation2d> freedomPathHeadings = new ArrayList<>();
        freedomPathHeadings.add(freedomPathTargetPosition.getRotation());
//        double freedomPathTrajectoryTime = freedomPathTrajectory.getTotalTimeSeconds();
//        for(int i = 0; i < freedomPathTrajectory.getStates().size(); i++){
//            freedomPathHeadings.add(Rotation2d.fromDegrees(
//                    freedomPathStartPosition.getRotation().getDegrees() * (freedomPathTrajectoryTime - freedomPathTrajectory.getStates().get(i).timeSeconds) / freedomPathTrajectoryTime +
//                            freedomPathTargetPosition.getRotation().getDegrees() * freedomPathTrajectory.getStates().get(i).timeSeconds / freedomPathTrajectoryTime
//            ));
//        }

//        System.out.println(freedomPathHeadings.get(0));
//        System.out.println(freedomPathHeadings.get(freedomPathHeadings.size()-1));
//        System.out.println(freedomPathHeadings);

        //Here's where your trajectory gets checked against the field
//        System.out.println("And survey says: "+testTrajectory(freedomPathTrajectory));

        freedomPathTrajectoryAction = new TrajectoryAction(freedomPathTrajectory, freedomPathHeadings);

        freedomPathTrajectoryAction.start();
    }

    /**
     * Called every loop
     */
    public void routine() throws AutoModeEndedException {
//        GreenLogger.log("FreedomPathing Running");

        // Run actions here:
        // e.g. runAction(new SeriesAction(new WaitAction(0.5), ...))

        // Run action, stop action on interrupt or done
        if (!freedomPathTrajectoryAction.isFinished()) {
            if (needsStop) {
                throw new AutoModeEndedException();
            }

            freedomPathTrajectoryAction.update();

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        else{
            freedomPathTrajectoryAction.done();
            done();
        }
    }

    /**
     * Standard cleanup end-procedure
     */
    protected void done() {
        robotState.isFreedomPathing = false;

        System.out.println("Started at "+freedomPathStartPosition);
        System.out.println("Hopefully ended at "+freedomPathTargetPosition);
        System.out.println("And it thinks it's at "+robotState.fieldToVehicle);

        GreenLogger.log("FreedomPath Done");
    }

    /**
     * Stops the auto path
     */
    public void stop() {
        freedomPathTrajectoryAction.done();
        robotState.isFreedomPathing = false;
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

            try {
                Thread.sleep(looperDtInMS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
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

    public double freedomPathMaxCalcMilli = 5;
    static double freedomPathBuffer = 8;
    static long startTime = 0;
//    static long totalTime = 0;
//    static int totalTries = 0;

    public Trajectory calculateFreedomPath(Pose2d freedomPathTargetPosition){
        return calculateFreedomPath(robotState.fieldToVehicle, freedomPathTargetPosition);
    }

    public Trajectory calculateFreedomPath(Pose2d freedomPathStartPosition, Pose2d freedomPathTargetPosition){
        freedomPathBuffer = SmartDashboard.getNumber("FreedomPath Waypoint Buffer", 10);

        if(freedomPathStartPosition.equals(freedomPathTargetPosition))
            return new Trajectory();
        robotState.freedomPathWaypoints.clear();
        robotState.freedomPathTrajectory = null;
        robotState.freedomPathCollisionStarts.clear();
        robotState.freedomPathCollisionEnds.clear();

        startTime = System.nanoTime()/1000000;

        if(fieldMap.getCurrentMap().checkPixelHasObjectOrOffMap((int)(freedomPathTargetPosition.getX()*mapResolution1DPerMeter), (int)(freedomPathTargetPosition.getY()*mapResolution1DPerMeter))) {
            if(robotState.freedomPathTrajectory != null)
                robotState.freedomPathTrajectoryChanged = true;
            return null;
        }
        if(fieldMap.getCurrentMap().checkPixelHasObjectOrOffMap((int)(freedomPathStartPosition.getX()*mapResolution1DPerMeter), (int)(freedomPathStartPosition.getY()*mapResolution1DPerMeter))) {
            if(robotState.freedomPathTrajectory != null)
                robotState.freedomPathTrajectoryChanged = true;
            return null;
        }

        Rotation2d startDirection = Rotation2d.fromRadians(Math.atan2(freedomPathTargetPosition.getY() - freedomPathStartPosition.getY(), freedomPathTargetPosition.getX() - freedomPathStartPosition.getX()));

        double velocity = 0;
        if(robotState.robotChassis != null)
            //projection formula
            velocity =
                    ((robotState.robotChassis.vxMetersPerSecond * (freedomPathTargetPosition.getX() - freedomPathStartPosition.getX()))
                            +(robotState.robotChassis.vyMetersPerSecond * (freedomPathTargetPosition.getY() - freedomPathStartPosition.getY())))
                            /Math.pow(Math.hypot(freedomPathTargetPosition.getY() - freedomPathStartPosition.getY(), freedomPathTargetPosition.getX() - freedomPathStartPosition.getX()), 2)
                            *Math.hypot(freedomPathTargetPosition.getY() - freedomPathStartPosition.getY(), freedomPathTargetPosition.getX() - freedomPathStartPosition.getX());


        TrajectoryConfig config = new TrajectoryConfig(Drive.kPathFollowingMaxVelMeters, Drive.kPathFollowingMaxAccelMeters);
//        config.setStartVelocity(velocity);
//        config.setEndVelocity(robotState.robotVelocity);
//        config.setEndVelocity(Math.min(Drive.kPathFollowingMaxVelMeters/2, 3));

        ArrayList<WaypointTreeNode> branches = new ArrayList<>();
        branches.add(
                new WaypointTreeNode(
                        new Pose2d(freedomPathStartPosition.getTranslation(), startDirection),
                        new ArrayList<>(),
                        new Pose2d(freedomPathTargetPosition.getTranslation(), Rotation2d.fromRadians(Math.atan2(freedomPathTargetPosition.getY() - freedomPathStartPosition.getY(), freedomPathTargetPosition.getX() - freedomPathStartPosition.getX()))),
                        config,
                        new ArrayList<>(),
                        false
                )
        );

        while(!branches.isEmpty() && !branches.get(0).trajectoryCheck){
            if(System.nanoTime()/1000000-startTime > freedomPathMaxCalcMilli)
                return null;
//            long startTime2 = System.nanoTime();
            boolean foundWorkingPath;
            for(int i = 1; i < branches.size(); i++)
                if(branches.get(i) == null) {
                    branches.remove(i);
                    i--;
                }else if(branches.get(i-1).trajectoryCheck) {
                    branches.remove(i);
                    i--;
                }

            int currentBranchIndex = 0;

            if(robotState.printFreedomPathing) {
                robotState.freedomPathTrajectoryPossibilities.clear();
                if (!branches.get(currentBranchIndex).trajectoryCheck) {
//                robotState.freedomPathTrajectoryPossibilities.add(branches.get(currentBranchIndex).getTrajectory());
                    for (WaypointTreeNode node : branches) {
                        robotState.freedomPathTrajectoryPossibilities.add(node.getTrajectory());

                    }
                }
                robotState.freedomPathTrajectoryPossibilitiesChanged = true;
            }

            ArrayList<Translation2d> waypoints = branches.get(currentBranchIndex).getWaypoints();
            Trajectory bestGuessTrajectory = branches.get(currentBranchIndex).getTrajectory();

//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

            WaypointTreeNode baseBranch = branches.remove(currentBranchIndex);

            int[] tempNewWaypointNegativeLast = getWaypointLast(bestGuessTrajectory, true);
            if(tempNewWaypointNegativeLast != null) {
                double[] newWaypointNegative = new double[]{tempNewWaypointNegativeLast[0] / mapResolution1DPerMeter, tempNewWaypointNegativeLast[1] / mapResolution1DPerMeter};
                ArrayList<Translation2d> newWaypointsNegative = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointNegative, newWaypointsNegative, freedomPathStartPosition, freedomPathTargetPosition, config);
                addNewWaypointOptimizedMaybe(newWaypointNegative, newWaypointsNegative, freedomPathStartPosition, freedomPathTargetPosition);
                ArrayList<Boolean> newPathTraceNegative = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
                newPathTraceNegative.add(false);

                WaypointTreeNode newNodeNeg =
                        new WaypointTreeNode(
                                new Pose2d(
                                        freedomPathStartPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        newWaypointsNegative.get(0).getY()
                                                                - freedomPathStartPosition.getY(),
                                                        newWaypointsNegative.get(0).getX()
                                                                - freedomPathStartPosition.getX()
                                                )
                                        )
                                ),
                                newWaypointsNegative,
                                new Pose2d(
                                        freedomPathTargetPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        freedomPathTargetPosition.getY()
                                                                - newWaypointsNegative.get(newWaypointsNegative.size() - 1).getY(),
                                                        freedomPathTargetPosition.getX()
                                                                - newWaypointsNegative.get(newWaypointsNegative.size() - 1).getX()
                                                )
                                        )
                                ),
                                config,
                                newPathTraceNegative,
                                baseBranch.isBoundaryPathBranch()
                        );

                addBranch(branches, newNodeNeg);

                robotState.freedomPathWaypoints.add(new Pose2d(new Translation2d(newWaypointNegative[0], newWaypointNegative[1]), new Rotation2d()));
            }

//            int[] tempNewWaypointNegative = getWaypoint(bestGuessTrajectory, true);
//            if(tempNewWaypointNegative != null) {
//                double[] newWaypointNegative = new double[]{tempNewWaypointNegative[0] / mapResolution1DPerMeter, tempNewWaypointNegative[1] / mapResolution1DPerMeter};
//                ArrayList<Translation2d> newWaypointsNegative = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointNegative, newWaypointsNegative, freedomPathStartPosition, freedomPathTargetPosition, config);
//                ArrayList<Boolean> newPathTraceNegative = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
//                newPathTraceNegative.add(false);
//
//                WaypointTreeNode newNodeNeg =
//                        new WaypointTreeNode(
//                                new Pose2d(
//                                        freedomPathStartPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        newWaypointsNegative.get(0).getY()
//                                                                - freedomPathStartPosition.getY(),
//                                                        newWaypointsNegative.get(0).getX()
//                                                                - freedomPathStartPosition.getX()
//                                                )
//                                        )
//                                ),
//                                newWaypointsNegative,
//                                new Pose2d(
//                                        freedomPathTargetPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        freedomPathTargetPosition.getY()
//                                                                - newWaypointsNegative.get(newWaypointsNegative.size() - 1).getY(),
//                                                        freedomPathTargetPosition.getX()
//                                                                - newWaypointsNegative.get(newWaypointsNegative.size() - 1).getX()
//                                                )
//                                        )
//                                ),
//                                config,
//                                newPathTraceNegative,
//                                baseBranch.isBoundaryPathBranch()
//                        );
//                addBranch(branches, newNodeNeg);
//
//                robotState.freedomPathWaypoints.add(new Pose2d(new Translation2d(newWaypointNegative[0], newWaypointNegative[1]), new Rotation2d()));
//            }

            if(System.nanoTime()/1000000-startTime > freedomPathMaxCalcMilli)
                return null;

            int[] tempNewWaypointPositiveLast = getWaypointLast(bestGuessTrajectory, false);
            if(tempNewWaypointPositiveLast != null) {
                double[] newWaypointPositive = new double[]{tempNewWaypointPositiveLast[0] / mapResolution1DPerMeter, tempNewWaypointPositiveLast[1] / mapResolution1DPerMeter};
                ArrayList<Translation2d> newWaypointsPositive = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointPositive, newWaypointsPositive, freedomPathStartPosition, freedomPathTargetPosition, config);
                addNewWaypointOptimizedMaybe(newWaypointPositive, newWaypointsPositive, freedomPathStartPosition, freedomPathTargetPosition);
                ArrayList<Boolean> newPathTracePositive = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
                newPathTracePositive.add(true);
                WaypointTreeNode newNodePos =
                        new WaypointTreeNode(
                                new Pose2d(
                                        freedomPathStartPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        newWaypointsPositive.get(0).getY()
                                                                - freedomPathStartPosition.getY(),
                                                        newWaypointsPositive.get(0).getX()
                                                                - freedomPathStartPosition.getX()
                                                )
                                        )
                                ),
                                newWaypointsPositive,
                                new Pose2d(
                                        freedomPathTargetPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        freedomPathTargetPosition.getY()
                                                                - newWaypointsPositive.get(newWaypointsPositive.size() - 1).getY(),
                                                        freedomPathTargetPosition.getX()
                                                                - newWaypointsPositive.get(newWaypointsPositive.size() - 1).getX()
                                                )
                                        )
                                ),
                                config,
                                newPathTracePositive,
                                baseBranch.isBoundaryPathBranch()
                        );

                addBranch(branches, newNodePos);

                robotState.freedomPathWaypoints.add(new Pose2d(new Translation2d(newWaypointPositive[0], newWaypointPositive[1]), new Rotation2d()));
            }

//            int[] tempNewWaypointPositive = getWaypoint(bestGuessTrajectory, false);
//            if (tempNewWaypointPositive != null) {
//                double[] newWaypointPositive = new double[]{tempNewWaypointPositive[0] / mapResolution1DPerMeter, tempNewWaypointPositive[1] / mapResolution1DPerMeter};
//                ArrayList<Translation2d> newWaypointsPositive = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointPositive, newWaypointsPositive, freedomPathStartPosition, freedomPathTargetPosition, config);
//                ArrayList<Boolean> newPathTracePositive = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
//                newPathTracePositive.add(true);
//                WaypointTreeNode newNodePos =
//                        new WaypointTreeNode(
//                                new Pose2d(
//                                        freedomPathStartPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        newWaypointsPositive.get(0).getY()
//                                                                - freedomPathStartPosition.getY(),
//                                                        newWaypointsPositive.get(0).getX()
//                                                                - freedomPathStartPosition.getX()
//                                                )
//                                        )
//                                ),
//                                newWaypointsPositive,
//                                new Pose2d(
//                                        freedomPathTargetPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        freedomPathTargetPosition.getY()
//                                                                - newWaypointsPositive.get(newWaypointsPositive.size() - 1).getY(),
//                                                        freedomPathTargetPosition.getX()
//                                                                - newWaypointsPositive.get(newWaypointsPositive.size() - 1).getX()
//                                                )
//                                        )
//                                ),
//                                config,
//                                newPathTracePositive,
//                                baseBranch.isBoundaryPathBranch()
//                        );
//                addBranch(branches, newNodePos);
//
//                robotState.freedomPathWaypoints.add(new Pose2d(new Translation2d(newWaypointPositive[0], newWaypointPositive[1]), new Rotation2d()));
//            }
//            totalTime += System.nanoTime()-startTime2;
//            totalTries++;
//
//            System.out.println("time taken: "+((System.nanoTime()-startTime2)/1000000)+", average: "+(totalTime/totalTries/1000000));
        }

        robotState.freedomPathTrajectory = branches.isEmpty() ? null : branches.get(0).getTrajectory();
        robotState.freedomPathTrajectoryChanged = true;

        if(!branches.isEmpty())
            robotState.freedomPathInputWaypoints = new ArrayList<>(branches.get(0).waypoints.stream()
                    .map(b -> new Pose2d(b, new Rotation2d()))
                    .toList());

        return branches.isEmpty() ? null : branches.get(0).getTrajectory();
    }

    //TODO if we ever need to optimize pathing, then optimize this, it's just a class to determine where in the order of waypoints would the new one be added, the slowdown is from generating so many trajectories using the TrajectoryGenerator, there should be ways around using it but I'm to lazy for now(1/20/2025)
    private void addNewWaypoint(double[] newWaypoint, List<Translation2d> waypoints, Pose2d startPos, Pose2d endPos, TrajectoryConfig config){
        int waypointsBeforeSize = waypoints.size();
        Translation2d newFirstWaypoint = new Translation2d(newWaypoint[0], newWaypoint[1]);
        Translation2d newLastWaypoint = waypoints.size() > 0 ? waypoints.get(waypoints.size()-1) : new Translation2d(newWaypoint[0], newWaypoint[1]);
        waypoints.add(0, new Translation2d(newWaypoint[0], newWaypoint[1]));
        double bestTime = TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPos.getTranslation(), Rotation2d.fromRadians(Math.atan2(newFirstWaypoint.getY()-startPos.getY(), newFirstWaypoint.getX()-startPos.getX()))),
                waypoints,
                new Pose2d(endPos.getTranslation(), Rotation2d.fromRadians(Math.atan2(endPos.getY()-newLastWaypoint.getY(), endPos.getX()-newLastWaypoint.getX()))),
                config
        ).getTotalTimeSeconds();
        waypoints.remove(0);
        int bestIndex = 0;

        for(int i = 0; i < waypoints.size(); i++){
            newFirstWaypoint = waypoints.get(0);
            newLastWaypoint = waypoints.get(waypoints.size()-1);

            double newTime;

            if (i == waypoints.size() - 1) {
                newLastWaypoint = new Translation2d(newWaypoint[0], newWaypoint[1]);
                waypoints.add(new Translation2d(newWaypoint[0], newWaypoint[1]));
                newTime = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(startPos.getTranslation(), Rotation2d.fromRadians(Math.atan2(newFirstWaypoint.getY() - startPos.getY(), newFirstWaypoint.getX() - startPos.getX()))),
                        waypoints,
                        new Pose2d(endPos.getTranslation(), Rotation2d.fromRadians(Math.atan2(endPos.getY() - newLastWaypoint.getY(), endPos.getX() - newLastWaypoint.getX()))),
                        config
                ).getTotalTimeSeconds();
                waypoints.remove(waypoints.size() - 1);
            } else {
                waypoints.add(i + 1, new Translation2d(newWaypoint[0], newWaypoint[1]));
                newTime = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(startPos.getTranslation(), Rotation2d.fromRadians(Math.atan2(newFirstWaypoint.getY() - startPos.getY(), newFirstWaypoint.getX() - startPos.getX()))),
                        waypoints,
                        new Pose2d(endPos.getTranslation(), Rotation2d.fromRadians(Math.atan2(endPos.getY() - newLastWaypoint.getY(), endPos.getX() - newLastWaypoint.getX()))),
                        config
                ).getTotalTimeSeconds();
                waypoints.remove(i + 1);
            }

            if (bestTime > newTime) {
                bestIndex = i + 1;
                bestTime = newTime;
            }
        }

        waypoints.add(bestIndex, new Translation2d(newWaypoint[0], newWaypoint[1]));
    }

    private void addNewWaypointOptimizedMaybe(double[] newWaypoint, List<Translation2d> waypoints, Pose2d startPos, Pose2d endPos){
        Translation2d newWaypointTranslation = new Translation2d(newWaypoint[0], newWaypoint[1]);
        if(waypoints.isEmpty()){
            waypoints.add(newWaypointTranslation);
            return;
        }

        int bestIndex = 0;
        double bestDistance = getPathDistanceLossFromAddingNewWaypoint(newWaypointTranslation, startPos.getTranslation(), waypoints.get(0));

        for(int i = 1; i <= waypoints.size(); i++){
            if(i == waypoints.size()){
                if(bestDistance > getPathDistanceLossFromAddingNewWaypoint(newWaypointTranslation, waypoints.get(i-1), endPos.getTranslation())){
                    bestDistance = getPathDistanceLossFromAddingNewWaypoint(newWaypointTranslation, waypoints.get(i-1), endPos.getTranslation());
                    bestIndex = waypoints.size();
                }
            } else{
                if(bestDistance > getPathDistanceLossFromAddingNewWaypoint(newWaypointTranslation, waypoints.get(i-1), waypoints.get(i))){
                    bestDistance = getPathDistanceLossFromAddingNewWaypoint(newWaypointTranslation, waypoints.get(i-1), waypoints.get(i));
                    bestIndex = i;
                }
            }
        }

        waypoints.add(bestIndex, newWaypointTranslation);
    }

    private double getPathDistanceLossFromAddingNewWaypoint(Translation2d newWaypoint, Translation2d waypoint, Translation2d consecutiveWaypoint){
        return (newWaypoint.getDistance(waypoint) + newWaypoint.getDistance(consecutiveWaypoint)) - waypoint.getDistance(consecutiveWaypoint);
    }

    private int[] getWaypoint(Trajectory bestGuessTrajectory, boolean makeNegative) {
        TimestampTranslation2d startCollision = returnCollisionStart(bestGuessTrajectory);
        TimestampTranslation2d endCollision = returnCollisionEnd(bestGuessTrajectory, startCollision);

        robotState.freedomPathCollisionStarts.add(new Pose2d(startCollision.getTranslation2d().times(1/mapResolution1DPerMeter), new Rotation2d()));
        robotState.freedomPathCollisionEnds.add(new Pose2d(endCollision.getTranslation2d().times(1/mapResolution1DPerMeter), new Rotation2d()));

        Translation2d startToEndTranspose = endCollision.getTranslation2d().minus(startCollision.getTranslation2d());

        int[] newWaypoint;

        int[] startNewCollision = new int[]{(int) startCollision.getTranslation2d().getX(), (int) startCollision.getTranslation2d().getY()};
        int[] endNewCollision = new int[]{(int) endCollision.getTranslation2d().getX(), (int) endCollision.getTranslation2d().getY()};

        ArrayList<Integer> pastCollisionPointHashes = new ArrayList<>();

        try {
            while (true) {
//            try {
//                Thread.sleep(100);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

                int[] collisionPoint;

                if (makeNegative)
                    collisionPoint =
                            Bresenham.drawPerpLineMinusOnePixelNegative(
                                    fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            ); //TODO fix the fact that im only perping "negatively"
                else
                    collisionPoint =
                            Bresenham.drawPerpLineMinusOnePixelPositive(
                                    fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            ); //TODO fix the fact that im only perping "negatively"

                if (collisionPoint == null)
                    return null;

                robotState.freedomPathWaypoints.add(new Pose2d(new Translation2d(collisionPoint[0] / mapResolution1DPerMeter, collisionPoint[1] / mapResolution1DPerMeter), new Rotation2d()));

                int[] possibleStartNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] - (int) (startToEndTranspose.getX() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] - (int) (startToEndTranspose.getY() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                true
                        );
                int[] possibleEndNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] + (int) (startToEndTranspose.getX() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] + (int) (startToEndTranspose.getY() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                true
                        );

                if (Arrays.equals(startNewCollision, endNewCollision)) {
                    newWaypoint = startNewCollision;
                    break;
                } else if (Arrays.equals(startNewCollision, possibleStartNewCollision) && Arrays.equals(endNewCollision, possibleEndNewCollision)) {
                    newWaypoint = startNewCollision;
                    break;
                } else if (checkCollisions(pastCollisionPointHashes, collisionPoint)) {
                    newWaypoint = startNewCollision;
//                System.out.println("AAAAHHHHHHH FreedomPathAlgorithm NOT DOING GOOD");
                    break;
                } //TODO if this ever actually triggers we need to revamp the system so that it...doesn't, this is basically just a botch solution to a really bad problem we may or may not have
                //TODO UPDATE: well now its used a lot and works soooo...ig it's a feature???
                else {
                    startNewCollision = possibleStartNewCollision;
                    endNewCollision = possibleEndNewCollision;
                }
                pastCollisionPointHashes.add(Arrays.hashCode(collisionPoint));
            }
        } catch (NullPointerException e){
            return null;
        }

        if(makeNegative){
            newWaypoint[0] -= (int)(freedomPathBuffer *Math.cos(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
            newWaypoint[1] -= (int)(freedomPathBuffer *Math.sin(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
        } else{
            newWaypoint[0] -= (int)(freedomPathBuffer *Math.cos(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
            newWaypoint[1] -= (int)(freedomPathBuffer *Math.sin(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
        }

        return newWaypoint;
    }

    private int[] getWaypointLast(Trajectory bestGuessTrajectory, boolean makeNegative) {
        TimestampTranslation2d startCollision = returnCollisionStartLast(bestGuessTrajectory);
        TimestampTranslation2d endCollision = returnCollisionEndLast(bestGuessTrajectory, startCollision);

        robotState.freedomPathCollisionStarts.add(new Pose2d(startCollision.getTranslation2d().times(1/mapResolution1DPerMeter), new Rotation2d()));
        robotState.freedomPathCollisionEnds.add(new Pose2d(endCollision.getTranslation2d().times(1/mapResolution1DPerMeter), new Rotation2d()));

        Translation2d startToEndTranspose = endCollision.getTranslation2d().minus(startCollision.getTranslation2d());

        if (startToEndTranspose.getX() == 0 && startToEndTranspose.getY() == 0)
            return null;

        int[] newWaypoint;

        int[] startNewCollision = new int[]{(int) startCollision.getTranslation2d().getX(), (int) startCollision.getTranslation2d().getY()};
        int[] endNewCollision = new int[]{(int) endCollision.getTranslation2d().getX(), (int) endCollision.getTranslation2d().getY()};

        ArrayList<Integer> pastCollisionPointHashes = new ArrayList<>();

        try {
            while (true) {
//            try {
//                Thread.sleep(100);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

                int[] collisionPoint;

                if (makeNegative)
                    collisionPoint =
                            Bresenham.drawPerpLineMinusOnePixelNegative(
                                    fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            );
                else
                    collisionPoint =
                            Bresenham.drawPerpLineMinusOnePixelPositive(
                                    fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            );

                if (collisionPoint == null)
                    return null;

                robotState.freedomPathWaypoints.add(new Pose2d(new Translation2d(collisionPoint[0] / mapResolution1DPerMeter, collisionPoint[1] / mapResolution1DPerMeter), new Rotation2d()));

                int[] possibleStartNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] - (int) (startToEndTranspose.getX() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] - (int) (startToEndTranspose.getY() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                true
                        );
                int[] possibleEndNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] + (int) (startToEndTranspose.getX() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] + (int) (startToEndTranspose.getY() * (Math.max(fieldMap.getCurrentMap().getMapX(), fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                true
                        );

                if (Arrays.equals(startNewCollision, endNewCollision)) {
                    newWaypoint = endNewCollision;
                    break;
                } else if (Arrays.equals(startNewCollision, possibleStartNewCollision) && Arrays.equals(endNewCollision, possibleEndNewCollision)) {
                    newWaypoint = endNewCollision;
                    break;
                } else if (checkCollisions(pastCollisionPointHashes, collisionPoint)) {
                    newWaypoint = endNewCollision;//                System.out.println("AAAAHHHHHHH FreedomPathAlgorithm NOT DOING GOOD");
                    break;
                } //TODO if this ever actually triggers we need to revamp the system so that it...doesn't, this is basically just a botch solution to a really bad problem we may or may not have
                //TODO UPDATE: well now its used a lot and works soooo...ig it's a feature???
                else {
                    startNewCollision = possibleStartNewCollision;
                    endNewCollision = possibleEndNewCollision;}
                pastCollisionPointHashes.add(Arrays.hashCode(collisionPoint));
            }
        } catch (NullPointerException e){
            return null;
        }

        if(makeNegative){
            newWaypoint[0] -= (int)(freedomPathBuffer*Math.cos(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
            newWaypoint[1] -= (int)(freedomPathBuffer*Math.sin(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
        } else{
            newWaypoint[0] -= (int)(freedomPathBuffer*Math.cos(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
            newWaypoint[1] -= (int)(freedomPathBuffer*Math.sin(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
        }

        return newWaypoint;
    }

    private boolean checkCollisions(List<Integer> pastCollisionPointHashes, int[] collisionPoint){
        int collisionPointHash = Arrays.hashCode(collisionPoint);
        for(int currentCollisionPointHash : pastCollisionPointHashes)
            if(collisionPointHash == currentCollisionPointHash)
                return true;
        return false;
    }

    private void addBranch(List<WaypointTreeNode> branches, WaypointTreeNode branch){
        for (int i = 0; i <= branches.size(); i++)
            if (i == branches.size()) {
                branches.add(branch);
                break;
            } else if (branch.getTrajectoryTime() < branches.get(i).getTrajectoryTime()) {
                branches.add(i, branch);
                break;
            }
    }

    public int[] getClosestValidPoint(int pointX, int pointY, FieldMap fieldMap){
        int currentPointX = pointX + Math.max(fieldMap.getMapX(), fieldMap.getMapY());
        int currentPointY = pointY + Math.max(fieldMap.getMapX(), fieldMap.getMapY());

        for(int i = 0; i < Math.hypot(currentPointX-pointX, currentPointY-pointY)*Math.pow(2, 0.5); i++){
            int checkPointX = pointX+i;
            int checkPointY = pointY+i;

            if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX-pointX, currentPointY-pointY) > Math.hypot(checkPointX-pointX, checkPointY-pointY)){
                currentPointX = checkPointX;
                currentPointY = checkPointY;
            }

            for(int i2 = 0; i2 < i*2; i2++){
                checkPointX--;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX-pointX, currentPointY-pointY) > Math.hypot(checkPointX-pointX, checkPointY-pointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }

            for(int i2 = 0; i2 < i*2; i2++){
                checkPointY--;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX-pointX, currentPointY-pointY) > Math.hypot(checkPointX-pointX, checkPointY-pointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }

            for(int i2 = 0; i2 < i*2; i2++){
                checkPointX++;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX-pointX, currentPointY-pointY) > Math.hypot(checkPointX-pointX, checkPointY-pointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }

            for(int i2 = 0; i2 < i*2; i2++){
                checkPointY++;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX-pointX, currentPointY-pointY) > Math.hypot(checkPointX-pointX, checkPointY-pointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }
        }

        System.out.println(currentPointX + " " + currentPointY);
        if(!fieldMap.checkPixelHasObjectOrOffMap(currentPointX, currentPointY)) {
            return new int[]{currentPointX, currentPointY};
        }else {
            System.out.println("FreedomPathAlgorithm has done a dumb in the getClosestValidPoint method and couldn't find a valid point");
            return null;
        }
    }

    class WaypointTreeNode {
        private final ArrayList<Translation2d> waypoints;
        private final ArrayList<Boolean> pathTrace;
        private final Trajectory trajectory;
        private final double trajectoryTime;
        private final boolean trajectoryCheck;
        private boolean boundaryPath = false;
        private final boolean boundaryPathBranch;

        WaypointTreeNode(Pose2d startPos, ArrayList<Translation2d> waypoints, Pose2d endPos, TrajectoryConfig config, ArrayList<Boolean> pathTrace, boolean boundaryPathBranch) {
            this.waypoints = (ArrayList<Translation2d>) waypoints.clone();
            this.pathTrace = (ArrayList<Boolean>) pathTrace.clone();
            trajectory = TrajectoryGenerator.generateTrajectory(
                    startPos,
                    waypoints,
                    endPos,
                    config
            );
            trajectoryTime = trajectory.getTotalTimeSeconds();
            trajectoryCheck = testTrajectory(trajectory);
            this.boundaryPathBranch = boundaryPathBranch;

            if(!pathTrace.isEmpty())
                a:{
                    boolean b = pathTrace.get(0);
                    for (boolean b2 : pathTrace){
                        if(b != b2){
                            break a;
                        }
                    }
                    boundaryPath = true;
                }
        }

        public boolean equals(WaypointTreeNode otherNode) {
            if (waypoints.equals(otherNode.getWaypoints())) {
                return true;
            }
            return false;
        }

        public ArrayList<Translation2d> getWaypoints() {
            return waypoints;
        }

        public Trajectory getTrajectory() {
            return trajectory;
        }

        public double getTrajectoryTime() {
            return trajectoryTime;
        }

        public ArrayList<Boolean> getPathTrace() {
            return pathTrace;
        }

        public boolean isBoundaryPath() {
            return boundaryPath;
        }

        public boolean isBoundaryPathBranch() {
            return boundaryPathBranch;
        }
    }
}