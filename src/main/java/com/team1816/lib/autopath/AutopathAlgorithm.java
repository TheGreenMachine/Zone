package com.team1816.lib.autopath;

import com.team1816.lib.hardware.factory.RobotFactory;
import com.team1816.lib.subsystems.drive.Drive;
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

public class AutopathAlgorithm {
    public static double autopathMaxCalcMilli = 5;
    static double autopathBuffer = 8;
    static long startTime = 0;
//    static long totalTime = 0;
//    static int totalTries = 0;

    public static Trajectory calculateAutopath(Pose2d autopathTargetPosition){
        return calculateAutopath(Autopath.robotState.fieldToVehicle, autopathTargetPosition);
    }

    public static Trajectory calculateAutopath(Pose2d autopathStartPosition, Pose2d autopathTargetPosition){
//        autopathBuffer = SmartDashboard.getNumber("Autopath Waypoint Buffer", 10);

        if(autopathStartPosition.equals(autopathTargetPosition))
            return new Trajectory();
        Autopath.robotState.autopathWaypoints.clear();
        Autopath.robotState.autopathTrajectory = null;
        Autopath.robotState.autopathCollisionStarts.clear();
        Autopath.robotState.autopathCollisionEnds.clear();

        startTime = System.nanoTime()/1000000;

        if(Autopath.fieldMap.getCurrentMap().checkPixelHasObjectOrOffMap((int)(autopathTargetPosition.getX()*Autopath.mapResolution1DPerMeter), (int)(autopathTargetPosition.getY()*Autopath.mapResolution1DPerMeter))) {
            if(Autopath.robotState.autopathTrajectory != null)
                Autopath.robotState.autopathTrajectoryChanged = true;
            return null;
        }
        if(Autopath.fieldMap.getCurrentMap().checkPixelHasObjectOrOffMap((int)(autopathStartPosition.getX()*Autopath.mapResolution1DPerMeter), (int)(autopathStartPosition.getY()*Autopath.mapResolution1DPerMeter))) {
            if(Autopath.robotState.autopathTrajectory != null)
                Autopath.robotState.autopathTrajectoryChanged = true;
            return null;
        }

        Rotation2d startDirection = Rotation2d.fromRadians(Math.atan2(autopathTargetPosition.getY() - autopathStartPosition.getY(), autopathTargetPosition.getX() - autopathStartPosition.getX()));

        double velocity = 0;
        if(Autopath.robotState.robotChassis != null)
            //projection formula
            velocity =
                ((Autopath.robotState.robotChassis.vxMetersPerSecond * (autopathTargetPosition.getX() - autopathStartPosition.getX()))
                +(Autopath.robotState.robotChassis.vyMetersPerSecond * (autopathTargetPosition.getY() - autopathStartPosition.getY())))
                /Math.pow(Math.hypot(autopathTargetPosition.getY() - autopathStartPosition.getY(), autopathTargetPosition.getX() - autopathStartPosition.getX()), 2)
                *Math.hypot(autopathTargetPosition.getY() - autopathStartPosition.getY(), autopathTargetPosition.getX() - autopathStartPosition.getX());


        TrajectoryConfig config = new TrajectoryConfig(Drive.kPathFollowingMaxVelMeters, Drive.kPathFollowingMaxAccelMeters);
        config.setStartVelocity(velocity);
//        config.setEndVelocity(Autopath.robotState.robotVelocity);
//        config.setEndVelocity(Math.min(Drive.kPathFollowingMaxVelMeters/2, 3));

        ArrayList<WaypointTreeNode> branches = new ArrayList<>();
        branches.add(
                new WaypointTreeNode(
                        new Pose2d(autopathStartPosition.getTranslation(), startDirection),
                        new ArrayList<>(),
                        new Pose2d(autopathTargetPosition.getTranslation(), Rotation2d.fromRadians(Math.atan2(autopathTargetPosition.getY() - autopathStartPosition.getY(), autopathTargetPosition.getX() - autopathStartPosition.getX()))),
                        config,
                        new ArrayList<>(),
                        false
                )
        );

        while(!branches.isEmpty() && !branches.get(0).trajectoryCheck){
            if(System.nanoTime()/1000000-startTime > autopathMaxCalcMilli)
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

            if(Autopath.robotState.printAutopathing) {
                Autopath.robotState.autopathTrajectoryPossibilities.clear();
                if (!branches.get(currentBranchIndex).trajectoryCheck) {
//                Autopath.robotState.autopathTrajectoryPossibilities.add(branches.get(currentBranchIndex).getTrajectory());
                    for (WaypointTreeNode node : branches) {
                        Autopath.robotState.autopathTrajectoryPossibilities.add(node.getTrajectory());

                    }
                }
                Autopath.robotState.autopathTrajectoryPossibilitiesChanged = true;
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
                double[] newWaypointNegative = new double[]{tempNewWaypointNegativeLast[0] / Autopath.mapResolution1DPerMeter, tempNewWaypointNegativeLast[1] / Autopath.mapResolution1DPerMeter};
                ArrayList<Translation2d> newWaypointsNegative = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointNegative, newWaypointsNegative, autopathStartPosition, autopathTargetPosition, config);
                addNewWaypointOptimizedMaybe(newWaypointNegative, newWaypointsNegative, autopathStartPosition, autopathTargetPosition);
                ArrayList<Boolean> newPathTraceNegative = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
                newPathTraceNegative.add(false);

                WaypointTreeNode newNodeNeg =
                        new WaypointTreeNode(
                                new Pose2d(
                                        autopathStartPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        newWaypointsNegative.get(0).getY()
                                                                - autopathStartPosition.getY(),
                                                        newWaypointsNegative.get(0).getX()
                                                                - autopathStartPosition.getX()
                                                )
                                        )
                                ),
                                newWaypointsNegative,
                                new Pose2d(
                                        autopathTargetPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        autopathTargetPosition.getY()
                                                                - newWaypointsNegative.get(newWaypointsNegative.size() - 1).getY(),
                                                        autopathTargetPosition.getX()
                                                                - newWaypointsNegative.get(newWaypointsNegative.size() - 1).getX()
                                                )
                                        )
                                ),
                                config,
                                newPathTraceNegative,
                                baseBranch.isBoundaryPathBranch()
                        );

                addBranch(branches, newNodeNeg);

                Autopath.robotState.autopathWaypoints.add(new Pose2d(new Translation2d(newWaypointNegative[0], newWaypointNegative[1]), new Rotation2d()));
            }

//            int[] tempNewWaypointNegative = getWaypoint(bestGuessTrajectory, true);
//            if(tempNewWaypointNegative != null) {
//                double[] newWaypointNegative = new double[]{tempNewWaypointNegative[0] / Autopath.mapResolution1DPerMeter, tempNewWaypointNegative[1] / Autopath.mapResolution1DPerMeter};
//                ArrayList<Translation2d> newWaypointsNegative = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointNegative, newWaypointsNegative, autopathStartPosition, autopathTargetPosition, config);
//                ArrayList<Boolean> newPathTraceNegative = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
//                newPathTraceNegative.add(false);
//
//                WaypointTreeNode newNodeNeg =
//                        new WaypointTreeNode(
//                                new Pose2d(
//                                        autopathStartPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        newWaypointsNegative.get(0).getY()
//                                                                - autopathStartPosition.getY(),
//                                                        newWaypointsNegative.get(0).getX()
//                                                                - autopathStartPosition.getX()
//                                                )
//                                        )
//                                ),
//                                newWaypointsNegative,
//                                new Pose2d(
//                                        autopathTargetPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        autopathTargetPosition.getY()
//                                                                - newWaypointsNegative.get(newWaypointsNegative.size() - 1).getY(),
//                                                        autopathTargetPosition.getX()
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
//                Autopath.robotState.autopathWaypoints.add(new Pose2d(new Translation2d(newWaypointNegative[0], newWaypointNegative[1]), new Rotation2d()));
//            }

            if(System.nanoTime()/1000000-startTime > autopathMaxCalcMilli)
                return null;

            int[] tempNewWaypointPositiveLast = getWaypointLast(bestGuessTrajectory, false);
            if(tempNewWaypointPositiveLast != null) {
                double[] newWaypointPositive = new double[]{tempNewWaypointPositiveLast[0] / Autopath.mapResolution1DPerMeter, tempNewWaypointPositiveLast[1] / Autopath.mapResolution1DPerMeter};
                ArrayList<Translation2d> newWaypointsPositive = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointPositive, newWaypointsPositive, autopathStartPosition, autopathTargetPosition, config);
                addNewWaypointOptimizedMaybe(newWaypointPositive, newWaypointsPositive, autopathStartPosition, autopathTargetPosition);
                ArrayList<Boolean> newPathTracePositive = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
                newPathTracePositive.add(true);
                WaypointTreeNode newNodePos =
                        new WaypointTreeNode(
                                new Pose2d(
                                        autopathStartPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        newWaypointsPositive.get(0).getY()
                                                                - autopathStartPosition.getY(),
                                                        newWaypointsPositive.get(0).getX()
                                                                - autopathStartPosition.getX()
                                                )
                                        )
                                ),
                                newWaypointsPositive,
                                new Pose2d(
                                        autopathTargetPosition.getTranslation(),
                                        Rotation2d.fromRadians(
                                                Math.atan2(
                                                        autopathTargetPosition.getY()
                                                                - newWaypointsPositive.get(newWaypointsPositive.size() - 1).getY(),
                                                        autopathTargetPosition.getX()
                                                                - newWaypointsPositive.get(newWaypointsPositive.size() - 1).getX()
                                                )
                                        )
                                ),
                                config,
                                newPathTracePositive,
                                baseBranch.isBoundaryPathBranch()
                        );

                addBranch(branches, newNodePos);

                Autopath.robotState.autopathWaypoints.add(new Pose2d(new Translation2d(newWaypointPositive[0], newWaypointPositive[1]), new Rotation2d()));
            }

//            int[] tempNewWaypointPositive = getWaypoint(bestGuessTrajectory, false);
//            if (tempNewWaypointPositive != null) {
//                double[] newWaypointPositive = new double[]{tempNewWaypointPositive[0] / Autopath.mapResolution1DPerMeter, tempNewWaypointPositive[1] / Autopath.mapResolution1DPerMeter};
//                ArrayList<Translation2d> newWaypointsPositive = (ArrayList<Translation2d>) waypoints.clone();
//                addNewWaypoint(newWaypointPositive, newWaypointsPositive, autopathStartPosition, autopathTargetPosition, config);
//                ArrayList<Boolean> newPathTracePositive = (ArrayList<Boolean>) baseBranch.getPathTrace().clone();
//                newPathTracePositive.add(true);
//                WaypointTreeNode newNodePos =
//                        new WaypointTreeNode(
//                                new Pose2d(
//                                        autopathStartPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        newWaypointsPositive.get(0).getY()
//                                                                - autopathStartPosition.getY(),
//                                                        newWaypointsPositive.get(0).getX()
//                                                                - autopathStartPosition.getX()
//                                                )
//                                        )
//                                ),
//                                newWaypointsPositive,
//                                new Pose2d(
//                                        autopathTargetPosition.getTranslation(),
//                                        Rotation2d.fromRadians(
//                                                Math.atan2(
//                                                        autopathTargetPosition.getY()
//                                                                - newWaypointsPositive.get(newWaypointsPositive.size() - 1).getY(),
//                                                        autopathTargetPosition.getX()
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
//                Autopath.robotState.autopathWaypoints.add(new Pose2d(new Translation2d(newWaypointPositive[0], newWaypointPositive[1]), new Rotation2d()));
//            }
//            totalTime += System.nanoTime()-startTime2;
//            totalTries++;
//
//            System.out.println("time taken: "+((System.nanoTime()-startTime2)/1000000)+", average: "+(totalTime/totalTries/1000000));
        }

        Autopath.robotState.autopathTrajectory = branches.isEmpty() ? null : branches.get(0).getTrajectory();
        Autopath.robotState.autopathTrajectoryChanged = true;

        if(!branches.isEmpty())
            Autopath.robotState.autopathInputWaypoints = new ArrayList<>(branches.get(0).waypoints.stream()
                .map(b -> new Pose2d(b, new Rotation2d()))
                .toList());

        return branches.isEmpty() ? null : branches.get(0).getTrajectory();
    }

    //TODO if we ever need to optimize pathing, then optimize this, it's just a class to determine where in the order of waypoints would the new one be added, the slowdown is from generating so many trajectories using the TrajectoryGenerator, there should be ways around using it but I'm to lazy for now(1/20/2025)
    private static void addNewWaypoint(double[] newWaypoint, List<Translation2d> waypoints, Pose2d startPos, Pose2d endPos, TrajectoryConfig config){
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

    private static void addNewWaypointOptimizedMaybe(double[] newWaypoint, List<Translation2d> waypoints, Pose2d startPos, Pose2d endPos){
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

    private static double getPathDistanceLossFromAddingNewWaypoint(Translation2d newWaypoint, Translation2d waypoint, Translation2d consecutiveWaypoint){
        return (newWaypoint.getDistance(waypoint) + newWaypoint.getDistance(consecutiveWaypoint)) - waypoint.getDistance(consecutiveWaypoint);
    }

    private static int[] getWaypoint(Trajectory bestGuessTrajectory, boolean makeNegative) {
        Autopath.TimestampTranslation2d startCollision = Autopath.returnCollisionStart(bestGuessTrajectory);
        Autopath.TimestampTranslation2d endCollision = Autopath.returnCollisionEnd(bestGuessTrajectory, startCollision);

        Autopath.robotState.autopathCollisionStarts.add(new Pose2d(startCollision.getTranslation2d().times(1/Autopath.mapResolution1DPerMeter), new Rotation2d()));
        Autopath.robotState.autopathCollisionEnds.add(new Pose2d(endCollision.getTranslation2d().times(1/Autopath.mapResolution1DPerMeter), new Rotation2d()));

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
                                    Autopath.fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            ); //TODO fix the fact that im only perping "negatively"
                else
                    collisionPoint =
                            Bresenham.drawPerpLineMinusOnePixelPositive(
                                    Autopath.fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            ); //TODO fix the fact that im only perping "negatively"

                if (collisionPoint == null)
                    return null;

                Autopath.robotState.autopathWaypoints.add(new Pose2d(new Translation2d(collisionPoint[0] / Autopath.mapResolution1DPerMeter, collisionPoint[1] / Autopath.mapResolution1DPerMeter), new Rotation2d()));

                int[] possibleStartNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                Autopath.fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] - (int) (startToEndTranspose.getX() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] - (int) (startToEndTranspose.getY() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                true
                        );
                int[] possibleEndNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                Autopath.fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] + (int) (startToEndTranspose.getX() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] + (int) (startToEndTranspose.getY() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
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
//                System.out.println("AAAAHHHHHHH AutopathAlgorithm NOT DOING GOOD");
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
            newWaypoint[0] -= (int)(autopathBuffer *Math.cos(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
            newWaypoint[1] -= (int)(autopathBuffer *Math.sin(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
        } else{
            newWaypoint[0] -= (int)(autopathBuffer *Math.cos(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
            newWaypoint[1] -= (int)(autopathBuffer *Math.sin(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
        }

        return newWaypoint;
    }

    private static int[] getWaypointLast(Trajectory bestGuessTrajectory, boolean makeNegative) {
        Autopath.TimestampTranslation2d startCollision = Autopath.returnCollisionStartLast(bestGuessTrajectory);
        Autopath.TimestampTranslation2d endCollision = Autopath.returnCollisionEndLast(bestGuessTrajectory, startCollision);

        Autopath.robotState.autopathCollisionStarts.add(new Pose2d(startCollision.getTranslation2d().times(1/Autopath.mapResolution1DPerMeter), new Rotation2d()));
        Autopath.robotState.autopathCollisionEnds.add(new Pose2d(endCollision.getTranslation2d().times(1/Autopath.mapResolution1DPerMeter), new Rotation2d()));

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
                                    Autopath.fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            );
                else
                    collisionPoint =
                            Bresenham.drawPerpLineMinusOnePixelPositive(
                                    Autopath.fieldMap.getCurrentMap(),
                                    startNewCollision[0],
                                    startNewCollision[1],
                                    endNewCollision[0],
                                    endNewCollision[1]
                            );

                if (collisionPoint == null)
                    return null;

                Autopath.robotState.autopathWaypoints.add(new Pose2d(new Translation2d(collisionPoint[0] / Autopath.mapResolution1DPerMeter, collisionPoint[1] / Autopath.mapResolution1DPerMeter), new Rotation2d()));

                int[] possibleStartNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                Autopath.fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] - (int) (startToEndTranspose.getX() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] - (int) (startToEndTranspose.getY() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                true
                        );
                int[] possibleEndNewCollision =
                        Bresenham.lineReturnCollisionInverted(
                                Autopath.fieldMap.getCurrentMap(),
                                collisionPoint[0],
                                collisionPoint[1],
                                collisionPoint[0] + (int) (startToEndTranspose.getX() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                collisionPoint[1] + (int) (startToEndTranspose.getY() * (Math.max(Autopath.fieldMap.getCurrentMap().getMapX(), Autopath.fieldMap.getCurrentMap().getMapY()) / startToEndTranspose.getNorm())),
                                true
                        );

                if (Arrays.equals(startNewCollision, endNewCollision)) {
                    newWaypoint = endNewCollision;
                    Autopath.robotState.autopath1++;
                    break;
                } else if (Arrays.equals(startNewCollision, possibleStartNewCollision) && Arrays.equals(endNewCollision, possibleEndNewCollision)) {
                    newWaypoint = endNewCollision;
                    Autopath.robotState.autopath2++;
                    break;
                } else if (checkCollisions(pastCollisionPointHashes, collisionPoint)) {
                    newWaypoint = endNewCollision;
                    Autopath.robotState.autopath3++;
//                System.out.println("AAAAHHHHHHH AutopathAlgorithm NOT DOING GOOD");
                    break;
                } //TODO if this ever actually triggers we need to revamp the system so that it...doesn't, this is basically just a botch solution to a really bad problem we may or may not have
                //TODO UPDATE: well now its used a lot and works soooo...ig it's a feature???
                else {
                    startNewCollision = possibleStartNewCollision;
                    endNewCollision = possibleEndNewCollision;
                    Autopath.robotState.autopath4++;
                }
                System.out.println(Autopath.robotState.autopath1+" "+Autopath.robotState.autopath2+" "+Autopath.robotState.autopath3+" "+Autopath.robotState.autopath4);

                pastCollisionPointHashes.add(Arrays.hashCode(collisionPoint));
            }
        } catch (NullPointerException e){
            return null;
        }

        if(makeNegative){
            newWaypoint[0] -= (int)(autopathBuffer*Math.cos(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
            newWaypoint[1] -= (int)(autopathBuffer*Math.sin(startToEndTranspose.getAngle().getRadians()+(Math.PI/2)));
        } else{
            newWaypoint[0] -= (int)(autopathBuffer*Math.cos(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
            newWaypoint[1] -= (int)(autopathBuffer*Math.sin(startToEndTranspose.getAngle().getRadians()-(Math.PI/2)));
        }

        return newWaypoint;
    }

    private static boolean checkCollisions(List<Integer> pastCollisionPointHashes, int[] collisionPoint){
        int collisionPointHash = Arrays.hashCode(collisionPoint);
        for(int currentCollisionPointHash : pastCollisionPointHashes)
            if(collisionPointHash == currentCollisionPointHash)
                return true;
        return false;
    }

    private static void addBranch(List<WaypointTreeNode> branches, WaypointTreeNode branch){
        for (int i = 0; i <= branches.size(); i++)
            if (i == branches.size()) {
                branches.add(branch);
                break;
            } else if (branch.getTrajectoryTime() < branches.get(i).getTrajectoryTime()) {
                branches.add(i, branch);
                break;
            }
    }

    private static int[] getClosestValidPoint(int pointX, int pointY, FieldMap fieldMap){
        int currentPointX = pointX + Math.max(fieldMap.getMapX(), fieldMap.getMapY());
        int currentPointY = pointY + Math.max(fieldMap.getMapX(), fieldMap.getMapY());

        for(int i = 0; i < Math.hypot(currentPointX, currentPointY)*Math.pow(2, 0.5); i++){
            int checkPointX = pointX+i;
            int checkPointY = pointY+i;

            if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX, currentPointY) > Math.hypot(checkPointX, checkPointY)){
                currentPointX = checkPointX;
                currentPointY = checkPointY;
            }

            for(int i2 = 0; i2 < i; i2++){
                checkPointX--;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX, currentPointY) > Math.hypot(checkPointX, checkPointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }

            for(int i2 = 0; i2 < i; i2++){
                checkPointY--;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX, currentPointY) > Math.hypot(checkPointX, checkPointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }

            for(int i2 = 0; i2 < i; i2++){
                checkPointX++;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX, currentPointY) > Math.hypot(checkPointX, checkPointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }

            for(int i2 = 0; i2 < i; i2++){
                checkPointY++;
                if(!fieldMap.checkPixelHasObjectOrOffMap(checkPointX, checkPointY) && Math.hypot(currentPointX, currentPointY) > Math.hypot(checkPointX, checkPointY)){
                    currentPointX = checkPointX;
                    currentPointY = checkPointY;
                }
            }
        }

        if(!fieldMap.checkPixelHasObjectOrOffMap(currentPointX, currentPointY))
            return new int[]{currentPointX, currentPointY};
        else {
            System.out.println("AutopathAlgorithm has done a dumb in the getClosestValidPoint method and couldn't find a valid point");
            return null;
        }
    }

    static class WaypointTreeNode {
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
            trajectoryCheck = Autopath.testTrajectory(trajectory);
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
