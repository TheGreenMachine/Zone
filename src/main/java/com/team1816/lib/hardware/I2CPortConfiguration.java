
package com.team1816.lib.hardware;

public class I2CPortConfiguration {

    public Object portID;
    public Object deviceID;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(I2CPortConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("portID");
        sb.append('=');
        sb.append(((this.portID == null)?"<null>":this.portID));
        sb.append(',');
        sb.append("deviceID");
        sb.append('=');
        sb.append(((this.deviceID == null)?"<null>":this.deviceID));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
