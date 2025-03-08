package com.team1816.lib.auto.modes;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.auto.actions.TrajectoryAction;
import com.team1816.lib.freedomPath.FreedomPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.SplineParameterizer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer;

import java.util.ArrayList;
import java.util.List;

public class FreedomPathMode extends AutoMode{
    Translation2d startTranslation = new Translation2d(/*2.6, 5.5*/2.751259,1.838709);
//    Translation2d startTranslation = new Translation2d(14, 4);
    //total avg: -0.15704970708570218 pos avg: 0.18805418060928844 ratio: 6.142624593037816
    FreedomPath freedomPather;

    public FreedomPathMode(){
        super();

        this.freedomPather = Injector.get(FreedomPath.class);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        Trajectory freedomPathTrajectory;

        int bIndex = 0;
        double percentSuccess = 0;

//        for(int b = 0; b <= 20; b++) {
//            SmartDashboard.putNumber("FreedomPath Waypoint Buffer", b);

        double beforeTime = System.nanoTime();

        robotState.freedomPathWaypointsSuccess.clear();
        robotState.freedomPathWaypointsFail.clear();

        int i3 = 0;
        long totalTime = 0;
        long highestTime = -1;
        robotState.printFreedomPathFieldTest = true;
        for (double i = 17.55; i >= 0; i -= 17.55 / 200.) {
            double lastTime = 0;
            for (double i2 = 0; i2 <= 8.05; i2 += 8.05 / 100.) {
//                if(lastTime < 25) {
//                    try {
//                        Thread.sleep((long) (25 - lastTime));
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                }

                long holdStartTime = System.nanoTime() / 1000;
                try {
                    i3++;
                    Trajectory test = freedomPather.calculateFreedomPath(new Pose2d(startTranslation, new Rotation2d()), new Pose2d(new Translation2d(i, i2), new Rotation2d(0)));
//                    Trajectory test = FreedomPathAlgorithm.calculateFreedomPath(new Pose2d(new Translation2d(i, i2), new Rotation2d(0)), new Pose2d(startTranslation, new Rotation2d()));
                    if (test != null && test.getStates().size() > 1) {
                        robotState.freedomPathWaypointsSuccess.add(new Pose2d(new Translation2d(i, i2), new Rotation2d()));
                    } else
                        robotState.freedomPathWaypointsFail.add(new Pose2d(new Translation2d(i, i2), new Rotation2d()));
                } catch (SplineParameterizer.MalformedSplineException |
                         TrajectoryParameterizer.TrajectoryGenerationException e) {
                    robotState.freedomPathWaypointsFail.add(new Pose2d(new Translation2d(i, i2), new Rotation2d()));
                }
                long holdEndTime = System.nanoTime() / 1000;
                totalTime += holdEndTime - holdStartTime;
                if (holdEndTime - holdStartTime > highestTime)
                    highestTime = holdEndTime - holdStartTime;
                lastTime = holdEndTime - holdStartTime;
//                    System.out.println(highestTime + ", " + totalTime / i3);
//                1017392, 2225 nonoptimized
//                119163, 906
            }
        }

//            if(percentSuccess < (double) robotState.freedomPathWaypointsSuccess.size() /(robotState.freedomPathWaypointsSuccess.size()+robotState.freedomPathWaypointsFail.size())){
//                bIndex = b;
//                percentSuccess = (double) robotState.freedomPathWaypointsSuccess.size() /(robotState.freedomPathWaypointsSuccess.size()+robotState.freedomPathWaypointsFail.size());
//            }
//
//            System.out.println("Best Buffer Thus Far = "+bIndex+" With a percentage of "+percentSuccess);
//        }
//        for(int i = 0; i < 10; i++)
//            System.out.println("Best Buffer = "+bIndex);

        freedomPathTrajectory = freedomPather.calculateFreedomPath(new Pose2d(new Translation2d(1.6, 5.5), Rotation2d.fromDegrees(90)));
//        freedomPathTrajectory = FreedomPathAlgorithm.calculateFreedomPath(new Pose2d(new Translation2d(1.3, 1.25), new Rotation2d(0)));


//        System.out.println("Time taken "+(System.nanoTime()-beforeTime)/1000000000);

        List<Rotation2d> freedomPathHeadings = new ArrayList<>();
        freedomPathHeadings.add(Rotation2d.fromDegrees(90));
//        double freedomPathTrajectoryTime = freedomPathTrajectory.getTotalTimeSeconds();
//        for(int i = 0; i < freedomPathTrajectory.getStates().size(); i++){
//            freedomPathHeadings.add(Rotation2d.fromDegrees(
//                    robotState.fieldToVehicle.getRotation().getDegrees() * ((freedomPathTrajectoryTime - freedomPathTrajectory.getStates().get(i).timeSeconds) / freedomPathTrajectoryTime) +
//                            90 * (freedomPathTrajectory.getStates().get(i).timeSeconds / freedomPathTrajectoryTime)
//            ));
//        }
//        List<Rotation2d> freedomPathHeadings = List.of(
//                Rotation2d.fromRadians(0),
//                Rotation2d.fromRadians(0.3),
//                Rotation2d.fromRadians(0.43),
//                Rotation2d.fromRadians(0.52),
//                Rotation2d.fromRadians(0.59),
//                Rotation2d.fromRadians(0.70),
//                Rotation2d.fromRadians(0.79),
//                Rotation2d.fromRadians(0.87),
//                Rotation2d.fromRadians(0.98),
//                Rotation2d.fromRadians(1.06),
//                Rotation2d.fromRadians(1.15),
//                Rotation2d.fromRadians(1.27),
//                Rotation2d.fromRadians(1.57)
//        );

        System.out.println(freedomPathHeadings);

        //Here's where your trajectory gets checked against the field
        System.out.println("And survey says: "+ freedomPather.testTrajectory(freedomPathTrajectory));

        TrajectoryAction freedomPathTrajectoryAction = new TrajectoryAction(freedomPathTrajectory, freedomPathHeadings);

        runAction(freedomPathTrajectoryAction);
    }

    public Pose2d getInitialPose() {
        return new Pose2d(startTranslation, robotState.allianceColor == Color.BLUE ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
    }
}
