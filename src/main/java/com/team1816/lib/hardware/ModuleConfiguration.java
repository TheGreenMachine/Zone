
package com.team1816.lib.hardware;

import java.util.Map;

public class ModuleConfiguration {

    public java.lang.String drive;
    public java.lang.String azimuth;
    public Map<String, Double> constants;
    public java.lang.String canCoder;
    public java.lang.Double xCoord;
    public java.lang.Double yCoord;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(ModuleConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("drive");
        sb.append('=');
        sb.append(((this.drive == null)?"<null>":this.drive));
        sb.append(',');
        sb.append("azimuth");
        sb.append('=');
        sb.append(((this.azimuth == null)?"<null>":this.azimuth));
        sb.append(',');
        sb.append("constants");
        sb.append('=');
        sb.append(((this.constants == null)?"<null>":this.constants));
        sb.append(',');
        sb.append("canCoder");
        sb.append('=');
        sb.append(((this.canCoder == null)?"<null>":this.canCoder));
        sb.append(',');
        sb.append("xCoord");
        sb.append('=');
        sb.append(((this.xCoord == null)?"<null>":this.xCoord));
        sb.append(',');
        sb.append("yCoord");
        sb.append('=');
        sb.append(((this.yCoord == null)?"<null>":this.yCoord));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
