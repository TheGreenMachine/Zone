package com.team1816.lib.auto.modes;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
import com.team1816.lib.auto.Color;
import com.team1816.lib.freedomPath.FreedomPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FreedomPathPointMode extends AutoMode{
    FreedomPath freedomPather;

    public FreedomPathPointMode(){
        super();

        this.freedomPather = Injector.get(FreedomPath.class);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        robotState.printFreedomPathFieldTest = true;
        for (double i = 17.55/2; i >= 0; i -= 17.55 / 200.) {
            for (double i2 = 0; i2 <= 8.05/2; i2 += 8.05 / 100.) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                int[] point = freedomPather.getClosestValidPoint((int)(i*100), (int)(i2*100), freedomPather.fieldMap.getCurrentMap());
                robotState.freedomPathWaypointsSuccess.add(new Pose2d(new Translation2d(point[0]/50., point[1]/50.), new Rotation2d()));
            }
        }
    }

    public Pose2d getInitialPose() {
        return new Pose2d(new Translation2d(4.7, 4), robotState.allianceColor == Color.BLUE ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
    }
}
