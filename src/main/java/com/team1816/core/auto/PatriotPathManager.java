package com.team1816.core.auto;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import com.team1816.core.configuration.Constants;
import com.team1816.core.states.RobotState;
import com.team1816.lib.Injector;
import com.team1816.lib.autopath.PatriotPath;
import com.team1816.lib.util.logUtil.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * A manager for choosing the target position of patriot paths
 */
@Singleton
public class PatriotPathManager {
    private final RobotState robotState;

    private final PatriotPath patriotPath;
    private final ColorManager colorManager;

    private final SendableChooser<DesiredLocation> locationChooser;
    private DesiredLocation desiredLocation;

    @Inject
    public PatriotPathManager() {
        this.robotState = Injector.get(RobotState.class);

        this.patriotPath = Injector.get(PatriotPath.class);
        this.colorManager = Injector.get(ColorManager.class);

        locationChooser = new SendableChooser<>();
        SmartDashboard.putData("Patriot Path Target", locationChooser);
        for (DesiredLocation desiredLocation : DesiredLocation.values()) {
            locationChooser.addOption(desiredLocation.name(), desiredLocation);
        }

        locationChooser.setDefaultOption(DesiredLocation.NOWHERE.name(), DesiredLocation.NOWHERE);

        desiredLocation = locationChooser.getSelected();
    }

    public void outputToSmartDashboard() {
        if (desiredLocation != null) {
            SmartDashboard.putString("PatriotLocationSelected", desiredLocation.name());
        }
    }

    public void update() {
        DesiredLocation selectedLocation = locationChooser.getSelected();
        boolean selectedLocationChanged = selectedLocation != desiredLocation;
        if (selectedLocationChanged) {
            GreenLogger.log("Changed desired location of Patriot Path from " + desiredLocation.name() + " to " + selectedLocation.name());
            desiredLocation = locationChooser.getSelected();
        }

        // Update logged target position if selected location is dynamic or has been changed
        if (desiredLocation.isDynamic || selectedLocationChanged || colorManager.isColorChanged()) {
            if (desiredLocation == DesiredLocation.NOWHERE) {
                robotState.field.getObject("patriotTarget").setPoses(List.of());
            } else {
                robotState.field.getObject("patriotTarget").setPoses(desiredLocation.getTargetPose());
            }
        }
    }

    public void start() {
        if (desiredLocation == DesiredLocation.NOWHERE) {
            GreenLogger.log("Attempted to start patriot path manager with no desired location.");
            return;
        }

        start(desiredLocation);
    }

    public void start(DesiredLocation desiredLocation) {
        patriotPath.start(desiredLocation.getTargetPose());
    }

    public enum DesiredLocation {
        // Default position
        NOWHERE(() -> null),

        // Dynamic positions
        NEAREST_CORAL_A(() -> {
            RobotState robotState = Injector.get(RobotState.class);
            return DesiredLocation.valueOf("CORAL_" + Util.getNearestReefSide(robotState.fieldToVehicle) + "A").getTargetPose();
        }, true),
        NEAREST_CORAL_MIDDLE(() -> {
            RobotState robotState = Injector.get(RobotState.class);
            return DesiredLocation.valueOf("CORAL_" + Util.getNearestReefSide(robotState.fieldToVehicle) + "M").getTargetPose();
        }, true),
        NEAREST_CORAL_B(() -> {
            RobotState robotState = Injector.get(RobotState.class);
            return DesiredLocation.valueOf("CORAL_" + Util.getNearestReefSide(robotState.fieldToVehicle) + "B").getTargetPose();
        }, true),

        // Non-dynamic positions
        TOP_FEEDER(() -> Util.flipIfNecessary(Constants.topFeederPose)),
        BOTTOM_FEEDER(() -> Util.flipIfNecessary(Constants.bottomFeederPose)),

        CORAL_1A(() -> Util.flipIfNecessary(Constants.reef1APose)),
        CORAL_1M(() -> Util.flipIfNecessary(Constants.reef1MPose)),
        CORAL_1B(() -> Util.flipIfNecessary(Constants.reef1BPose)),

        CORAL_2A(() -> Util.flipIfNecessary(Constants.reef2APose)),
        CORAL_2M(() -> Util.flipIfNecessary(Constants.reef2MPose)),
        CORAL_2B(() -> Util.flipIfNecessary(Constants.reef2BPose)),

        CORAL_3A(() -> Util.flipIfNecessary(Constants.reef3APose)),
        CORAL_3M(() -> Util.flipIfNecessary(Constants.reef3MPose)),
        CORAL_3B(() -> Util.flipIfNecessary(Constants.reef3BPose)),

        CORAL_4A(() -> Util.flipIfNecessary(Constants.reef4APose)),
        CORAL_4M(() -> Util.flipIfNecessary(Constants.reef4MPose)),
        CORAL_4B(() -> Util.flipIfNecessary(Constants.reef4BPose)),

        CORAL_5A(() -> Util.flipIfNecessary(Constants.reef5APose)),
        CORAL_5M(() -> Util.flipIfNecessary(Constants.reef5MPose)),
        CORAL_5B(() -> Util.flipIfNecessary(Constants.reef5BPose)),

        CORAL_6A(() -> Util.flipIfNecessary(Constants.reef6APose)),
        CORAL_6M(() -> Util.flipIfNecessary(Constants.reef6MPose)),
        CORAL_6B(() -> Util.flipIfNecessary(Constants.reef6BPose)),

        ;

        public Pose2d getTargetPose() {
            return target.get();
        }

        DesiredLocation(Supplier<Pose2d> target) {
            this(target, false);
        }

        DesiredLocation(Supplier<Pose2d> target, boolean isDynamic) {
            this.target = target;
            this.isDynamic = isDynamic;
        }

        private final Supplier<Pose2d> target;
        private final boolean isDynamic;
    }

    private static class Util {
        // A map of all the locations of the edge of reefs
        private static final Map<String, Translation2d> REEF_LOCATIONS = Map.of(
                "1", new Translation2d(4.936, 4.788),
                "2", new Translation2d(5.315, 4.020),
                "3", new Translation2d(4.926, 3.302),
                "4", new Translation2d(4.098, 3.302),
                "5", new Translation2d(3.610, 4.000),
                "6", new Translation2d(4.038, 4.788)
                );

        public static Pose2d flipIfNecessary(Pose2d pose) {
            return AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(pose) : pose;
        }

        public static String getNearestReefSide(Pose2d pose) {
            pose = flipIfNecessary(pose); // If robot is red, this flips it back to being blue. Kind of hacky, but efficient

            String minReef = null;
            double minDistance = Double.MAX_VALUE;

            for (Map.Entry<String, Translation2d> entry : REEF_LOCATIONS.entrySet()) {
                double distance = pose.getTranslation().getDistance(entry.getValue());
                if (distance < minDistance) {
                    minDistance = distance;
                    minReef = entry.getKey();
                }
            }

            return minReef;
        }
    }
}
