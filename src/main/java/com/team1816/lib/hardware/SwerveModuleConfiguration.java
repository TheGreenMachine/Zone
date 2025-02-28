
package com.team1816.lib.hardware;

import java.util.Map;

public class SwerveModuleConfiguration {

    public Map<String, ModuleConfiguration> modules;
    public Map<String, PIDSlotConfiguration> azimuthPID;
    public Map<String, PIDSlotConfiguration> drivePID;
    public Map<String, Double> constants;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(SwerveModuleConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("modules");
        sb.append('=');
        sb.append(((this.modules == null)?"<null>":this.modules));
        sb.append(',');
        sb.append("azimuthPID");
        sb.append('=');
        sb.append(((this.azimuthPID == null)?"<null>":this.azimuthPID));
        sb.append(',');
        sb.append("drivePID");
        sb.append('=');
        sb.append(((this.drivePID == null)?"<null>":this.drivePID));
        sb.append(',');
        sb.append("constants");
        sb.append('=');
        sb.append(((this.constants == null)?"<null>":this.constants));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
