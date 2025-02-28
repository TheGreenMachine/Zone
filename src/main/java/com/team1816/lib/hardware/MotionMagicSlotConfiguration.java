
package com.team1816.lib.hardware;
public class MotionMagicSlotConfiguration {

    public Double cruiseVelocity;
    public Double acceleration;
    public Double jerk;
    public Double expoKV;
    public Double expoKA;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(MotionMagicSlotConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("cruiseVelocity");
        sb.append('=');
        sb.append(((this.cruiseVelocity == null)?"<null>":this.cruiseVelocity));
        sb.append(',');
        sb.append("acceleration");
        sb.append('=');
        sb.append(((this.acceleration == null)?"<null>":this.acceleration));
        sb.append(',');
        sb.append("jerk");
        sb.append('=');
        sb.append(((this.jerk == null)?"<null>":this.jerk));
        sb.append(',');
        sb.append("expoKV");
        sb.append('=');
        sb.append(((this.expoKV == null)?"<null>":this.expoKV));
        sb.append(',');
        sb.append("expoKA");
        sb.append('=');
        sb.append(((this.expoKA == null)?"<null>":this.expoKA));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
