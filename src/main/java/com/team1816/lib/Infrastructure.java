package com.team1816.lib;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import com.team1816.lib.hardware.components.pcm.ICompressor;
import com.team1816.lib.hardware.components.sensor.IProximitySensor;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.wpilibj.PowerDistribution;

import java.util.List;

/**
 * Super-system housing compressor, pigeon, and power distribution
 */

@Singleton
public class Infrastructure {

    /**
     * Components
     */
    private static ICompressor compressor;
    private static PowerDistribution pd;
    public static List<IProximitySensor> proximitySensors;

    private static boolean compressorEnabled;
    private static boolean compressorIsOn = false;

    private double pitchOffset;
    private double rollOffset;

    /**
     * Instantiates the infrastructure with RobotFactory
     *
     * @param factory RobotFactory
     * @see RobotFactory
     */
    @Inject
    public Infrastructure(RobotFactory factory) {
        compressor = factory.getCompressor();
        pd = factory.getPd();
        compressorEnabled = factory.isCompressorEnabled();

        var frontLeft = factory.getProximitySensor("FLProximitySensor");
        var frontRight = factory.getProximitySensor("FRProximitySensor");
        var rearLeft = factory.getProximitySensor("RLProximitySensor");
        var rearRight = factory.getProximitySensor("RRProximitySensor");
        proximitySensors = List.of(frontLeft, frontRight, rearLeft, rearRight);
        pitchOffset = 0;
        rollOffset = 0;
    }

    /**
     * Returns the input voltage / bus voltage
     */
    public double getBusVoltage() {
        return pd.getVoltage();
    }

    /**
     * Starts the compressor. Not used now because the compressor turns on by default once the robot is enabled.
     */
    public void startCompressor() {
        if (compressorEnabled && !compressorIsOn) {
            compressor.enableDigital();
            compressorIsOn = true;
        }
    }

    /**
     * Stops the compressor
     *
     * @see Infrastructure#startCompressor()
     */
    public void stopCompressor() {
        if (compressorEnabled && compressorIsOn) {
            compressor.disable();
            compressorIsOn = false;
        }
    }

    /**
     * Resets the pigeon gyroscope based on a Rotation2d
     *
     * @param angle Rotation2d
     */



    /**
     * Returns the power distribution associated with the Infrastructure
     *
     * @return PowerDistribution
     * @see PowerDistribution
     */
    public PowerDistribution getPd() {
        return pd;
    }


    /**
     * Returns the maximal locus proximity of the drivetrain in relation to the floor
     *
     * @return maximumProximity
     */
    public double getMaximumProximity() {
        double maximumProximity = -1;
        for (int i = 0; i < 3; i++) {
            double proximity = proximitySensors.get(i).getProximity();
            if (proximity > maximumProximity && proximity < 80) {
                maximumProximity = proximity;
            }
        }
        return maximumProximity;
    }
}
