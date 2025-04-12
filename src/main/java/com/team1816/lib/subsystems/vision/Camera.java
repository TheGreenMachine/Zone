package com.team1816.lib.subsystems.vision;

import com.team1816.lib.Infrastructure;
import com.team1816.lib.subsystems.Subsystem;
import com.team1816.core.configuration.Constants;
import com.team1816.core.configuration.FieldConfig;
import com.team1816.core.states.RobotState;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@Singleton
public class Camera extends Subsystem{
    /**
     * Properties
     */
    private static final String NAME = "camera";
    private static final List<String> CAMS = List.of("Arducam_OV9281_USB_Camera", "Arducam_OV9281_USB_Camera (1)");
    private static final List<AprilTag> aprilTags = List.of(
            new AprilTag(20, new Pose3d(1, 0, 0.1525, new Rotation3d(0,0,Math.PI)))
    );
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
//            new AprilTagFieldLayout(aprilTags, 17.548, 8.052);
    private static final List<Transform3d> robotToCams = Constants.kCameraMountingOffset3Ds;

    /**
     * Components
     */
    private VisionSystemSim visionSim;
    private final List<PhotonCamera> cams = new ArrayList<>();
    private final List<PhotonPoseEstimator> photonEstimators = new ArrayList<>();

    /**
     * Logging
     */
    protected List<StructLogEntry<Pose2d>> visionPoseLoggers = new ArrayList<>();

    @Inject
    public Camera(Infrastructure inf, RobotState rs){
        super(NAME, inf, rs);
        visionSim = new VisionSystemSim("SimVision");
        visionSim.addAprilTags(kTagLayout);
        for (int i = 0; i < CAMS.size(); i++) {
            cams.add(new PhotonCamera(CAMS.get(i)));

            if (RobotBase.isSimulation()) {
                var cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
                cameraProp.setCalibError(0.35, 0.10);
                cameraProp.setFPS(15);
                cameraProp.setAvgLatencyMs(50);
                cameraProp.setLatencyStdDevMs(15);
                PhotonCameraSim cameraSim = new PhotonCameraSim(cams.get(i), cameraProp, kTagLayout);
                cameraSim.enableDrawWireframe(true);
                visionSim.addCamera(cameraSim, robotToCams.get(i));
            }
            photonEstimators.add(new PhotonPoseEstimator(kTagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCams.get(i)));
            photonEstimators.get(i).setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
            if (Constants.kLoggingRobot) {
                visionPoseLoggers.add(StructLogEntry.create(DataLogManager.getLog(), "Camera/visionPose" + i, Pose2d.struct));
            }
        }
    }

    public void setDriverMode(int camera, boolean driverMode){
        cams.get(camera).setDriverMode(driverMode);
    }

    /**
     * Updates the vision estimated positions from the cameras
     */
    public void updateVisionEstimatedPoses() {
        robotState.visionEstimatedPoses.clear();
        robotState.visionStdDevs.clear();
        Optional<EstimatedRobotPose> visionEst;
        List<Pose2d> seenTargetPoses = new ArrayList<>(List.of());
        boolean hasNewCameraResults = false;
        for (int i = 0; i < cams.size(); i++) {
            for (var change : cams.get(i).getAllUnreadResults()) {

                hasNewCameraResults = true;
                visionEst = photonEstimators.get(i).update(change);
                int finalI = i;
                visionEst.ifPresent(
                        est -> {
                            robotState.visionEstimatedPoses.add(est);
                            robotState.visionStdDevs.add(getEstimationStdDevs(est));
                            FieldConfig.field.getObject("vision" + finalI).setPose(est.estimatedPose.toPose2d());
                            if (Constants.kLoggingDrivetrain) {
                                visionPoseLoggers.get(finalI).append(est.estimatedPose.toPose2d());
                            }
                            for (PhotonTrackedTarget target : change.getTargets())
                                seenTargetPoses.add(est.estimatedPose.plus(robotToCams.get(finalI)).plus(target.bestCameraToTarget).toPose2d());
                        });
            }
//            FieldConfig.field.getObject("camera"+i).setPose(
//                    robotState.fieldToVehicle.plus(new Transform2d(robotToCams.get(i).getTranslation().toTranslation2d(), robotToCams.get(i).getRotation().toRotation2d()))
//            );
        }
//        if(hasNewCameraResults)
//            FieldConfig.field.getObject("visibleTargetPoses").setPoses(seenTargetPoses);
    }

    /**
     * The standard deviations of the estimated pose from {@link #updateVisionEstimatedPoses()}, for use
     * with {@link SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(
            EstimatedRobotPose estimatedPose) {
        // Ignore estimates that are too far off of our current estimate as they are
        // probably not correct
        if (Math.abs(robotState.fieldToVehicle.getRotation().getDegrees()
                - estimatedPose.estimatedPose.getRotation().toRotation2d().getDegrees()) >= 15 // Angle difference in degrees
                || robotState.fieldToVehicle.getTranslation().getDistance(
                        estimatedPose.estimatedPose.toPose2d().getTranslation()) >= 1.5 // Position difference in meters
        )
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        var estStdDevs = robotState.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;
        double closestDist = Double.MAX_VALUE;
        double lowestAmbiguity = 1;
        for (var tgt : estimatedPose.targetsUsed) {
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            double distance = tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
            avgDist += distance;
            if (distance < closestDist)
                closestDist = distance;
            double ambiguity = tgt.getPoseAmbiguity();
            if (ambiguity < lowestAmbiguity)
                lowestAmbiguity = ambiguity;
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = robotState.kMultiTagStdDevs;
        // Ignore estimates when the tags are too far from the camera or the ambiguities are all too high
        if (closestDist > 2 || lowestAmbiguity > 0.2)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        // Increase std devs based on (average) distance
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        // Set rotation std dev to max value because we trust our gyro over vision correction
//        estStdDevs.set(2, 0, Double.MAX_VALUE);

        return estStdDevs;
    }

    @Override
    public void readFromHardware() {
        if (RobotBase.isSimulation()) {
            visionSim.update(robotState.simActualFieldToVehicle);
        }
    }

    @Override
    public void writeToHardware() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean testSubsystem() {
        return false;
    }
}