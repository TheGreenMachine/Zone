
package com.team1816.lib.hardware;

import java.util.Map;

public class SensorConfiguration {

    public Map<String, I2CPortConfiguration> i2c;
    public Map<String, Integer> digitalInputs;
    public Map<String, Integer> analogInputs;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(SensorConfiguration.class.getName()).append('@').append(java.lang.Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("i2c");
        sb.append('=');
        sb.append(((this.i2c == null)?"<null>":this.i2c));
        sb.append(',');
        sb.append("digitalInputs");
        sb.append('=');
        sb.append(((this.digitalInputs == null)?"<null>":this.digitalInputs));
        sb.append(',');
        sb.append("analogInputs");
        sb.append('=');
        sb.append(((this.analogInputs == null)?"<null>":this.analogInputs));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
