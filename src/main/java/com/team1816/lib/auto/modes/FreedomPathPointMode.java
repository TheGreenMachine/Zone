package com.team1816.lib.auto.modes;

import com.team1816.lib.Injector;
import com.team1816.lib.auto.AutoModeEndedException;
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
                int[] point = freedomPather.getClosestValidPoint((int)(i*100), (int)(i2*100), freedomPather.fieldMapExpanded.getCurrentMap());
                robotState.freedomPathWaypointsSuccess.add(new Pose2d(new Translation2d(point[0]/100., point[1]/100.), new Rotation2d()));
            }
        }
    }
}
