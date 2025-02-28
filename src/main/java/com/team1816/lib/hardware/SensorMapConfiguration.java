
package com.team1816.lib.hardware;

public class SensorMapConfiguration {

    public SensorConfiguration colorSensors;
    public SensorConfiguration beamBreaks;
    public SensorConfiguration hallEffects;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(SensorMapConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("colorSensors");
        sb.append('=');
        sb.append(((this.colorSensors == null)?"<null>":this.colorSensors));
        sb.append(',');
        sb.append("beamBreaks");
        sb.append('=');
        sb.append(((this.beamBreaks == null)?"<null>":this.beamBreaks));
        sb.append(',');
        sb.append("hallEffects");
        sb.append('=');
        sb.append(((this.hallEffects == null)?"<null>":this.hallEffects));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
