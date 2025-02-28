
package com.team1816.lib.hardware;

import com.team1816.lib.hardware.components.motor.IGreenMotor.MotorType;

public class MotorConfiguration {

    public MotorType motorType;
    public String motorName;
    public Integer id;
    public Double currentLimit;
    public Double currentLimitThreshold;
    public Double currentLimitTriggerTime;
    public Boolean enableCurrentLimit;
    public Boolean invertMotor;
    public Boolean isLowSpeedCanBus;

    public MotionMagicSlotConfiguration motionMagic;

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(MotorConfiguration.class.getName()).append('@').append(Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("motorType");
        sb.append('=');
        sb.append(((this.motorType == null)?"<null>":this.motorType));
        sb.append(',');
        sb.append("motorName");
        sb.append('=');
        sb.append(((this.motorName == null)?"<null>":this.motorName));
        sb.append(',');
        sb.append("id");
        sb.append('=');
        sb.append(((this.id == null)?"<null>":this.id));
        sb.append(',');
        sb.append("currentLimit");
        sb.append('=');
        sb.append(((this.currentLimit == null)?"<null>":this.currentLimit));
        sb.append(',');
        sb.append("currentLimitThreshold");
        sb.append('=');
        sb.append(((this.currentLimitThreshold == null)?"<null>":this.currentLimitThreshold));
        sb.append(',');
        sb.append("currentLimitTriggerTime");
        sb.append('=');
        sb.append(((this.currentLimitTriggerTime == null)?"<null>":this.currentLimitTriggerTime));
        sb.append(',');
        sb.append("enableCurrentLimit");
        sb.append('=');
        sb.append(((this.enableCurrentLimit == null)?"<null>":this.enableCurrentLimit));
        sb.append(',');
        sb.append("invertMotor");
        sb.append('=');
        sb.append(((this.invertMotor == null)?"<null>":this.invertMotor));
        sb.append(',');
        sb.append("isLowSpeedCanBus");
        sb.append('=');
        sb.append(((this.isLowSpeedCanBus == null)?"<null>":this.isLowSpeedCanBus));
        sb.append(',');
        sb.append("motionMagic");
        sb.append('=');
        sb.append(((this.motionMagic == null)?"<null>":this.motionMagic));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
