
package com.team1816.lib.hardware;

public class DoubleSolenoidConfig {

    public Integer forward;
    public Integer reverse;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(DoubleSolenoidConfig.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("forward");
        sb.append('=');
        sb.append(((this.forward == null)?"<null>":this.forward));
        sb.append(',');
        sb.append("reverse");
        sb.append('=');
        sb.append(((this.reverse == null)?"<null>":this.reverse));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
