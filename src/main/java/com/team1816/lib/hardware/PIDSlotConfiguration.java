
package com.team1816.lib.hardware;

public class PIDSlotConfiguration {

    public Double kP;
    public Double kI;
    public Double kD;
    public Double kV;
    public Double kS;
    public Double kA;
    public Double kG;
    public String gravityType;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(PIDSlotConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("kP");
        sb.append('=');
        sb.append(((this.kP == null)?"<null>":this.kP));
        sb.append(',');
        sb.append("kI");
        sb.append('=');
        sb.append(((this.kI == null)?"<null>":this.kI));
        sb.append(',');
        sb.append("kD");
        sb.append('=');
        sb.append(((this.kD == null)?"<null>":this.kD));
        sb.append(',');
        sb.append("kV");
        sb.append('=');
        sb.append(((this.kV == null)?"<null>":this.kV));
        sb.append(',');
        sb.append("kS");
        sb.append('=');
        sb.append(((this.kS == null)?"<null>":this.kS));
        sb.append(',');
        sb.append("kA");
        sb.append('=');
        sb.append(((this.kA == null)?"<null>":this.kA));
        sb.append(',');
        sb.append("kG");
        sb.append('=');
        sb.append(((this.kG == null)?"<null>":this.kG));
        sb.append(',');
        sb.append("gravityType");
        sb.append('=');
        sb.append(((this.gravityType == null)?"<null>":this.gravityType));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
