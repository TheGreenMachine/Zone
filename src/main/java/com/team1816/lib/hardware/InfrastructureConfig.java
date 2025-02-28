
package com.team1816.lib.hardware;

import java.util.Map;
import javax.annotation.processing.Generated;
import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;



public class InfrastructureConfig {

    public java.lang.Integer pcmId;
    public Boolean pcmIsRev;
    public Boolean compressorEnabled;
    public java.lang.String canBusName;
    public Boolean pdIsRev;
    public java.lang.Integer pdId;
    public java.lang.Integer pigeonId;
    public Boolean isPigeon2;
    public Map<String, Integer> proximitySensors;

    @Override
    public java.lang.String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(InfrastructureConfig.class.getName()).append('@').append(java.lang.Integer.toHexString(System.identityHashCode(this))).append('[');
        sb.append("pcmId");
        sb.append('=');
        sb.append(((this.pcmId == null)?"<null>":this.pcmId));
        sb.append(',');
        sb.append("pcmIsRev");
        sb.append('=');
        sb.append(((this.pcmIsRev == null)?"<null>":this.pcmIsRev));
        sb.append(',');
        sb.append("compressorEnabled");
        sb.append('=');
        sb.append(((this.compressorEnabled == null)?"<null>":this.compressorEnabled));
        sb.append(',');
        sb.append("canBusName");
        sb.append('=');
        sb.append(((this.canBusName == null)?"<null>":this.canBusName));
        sb.append(',');
        sb.append("pdIsRev");
        sb.append('=');
        sb.append(((this.pdIsRev == null)?"<null>":this.pdIsRev));
        sb.append(',');
        sb.append("pdId");
        sb.append('=');
        sb.append(((this.pdId == null)?"<null>":this.pdId));
        sb.append(',');
        sb.append("pigeonId");
        sb.append('=');
        sb.append(((this.pigeonId == null)?"<null>":this.pigeonId));
        sb.append(',');
        sb.append("isPigeon2");
        sb.append('=');
        sb.append(((this.isPigeon2 == null)?"<null>":this.isPigeon2));
        sb.append(',');
        sb.append("proximitySensors");
        sb.append('=');
        sb.append(((this.proximitySensors == null)?"<null>":this.proximitySensors));
        sb.append(',');
        if (sb.charAt((sb.length()- 1)) == ',') {
            sb.setCharAt((sb.length()- 1), ']');
        } else {
            sb.append(']');
        }
        return sb.toString();
    }

}
