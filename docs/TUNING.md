# Tuning Guide

A guide on how to tune things on the robot.


## What I Just Tried to do to Tune the Elevator that Almost Worked
* Ran with DutyCycleOut (percent output) to make sure it was going the correct direction
* Ran in PositionDutyCycle with a kP of 1 (because the old elevator had it at 1) and all other PID values at 0, graphing position
* Changed the kD to 0.02 because it was spiking past the set positions before coming back down and that was the value for the old elevator
* Lowered the Peak Forward Duty Cycle to 0.4 and raised the Peak Reverse Duty Cycle to -0.4 because it was being sort of aggressive and to make sure it didn't totally break if I messed something up
* Switched to MotionMagicExpoVoltage and used 0.1 for expoKV and 0.2 for expoKA because that was what we had for the old elevator
* At this point, it seemed like it wasn't going far enough going either up or down, so I switched to voltage out and slowly increased the voltage to see what it needed to be for the elevator to barely start moving up, which was 0.7, so I switched back to MotionMagicExpoVoltage and put in 0.7 for kG
* Now it was getting to the right position going up but not going low enough going down, so I changed kS to 0.1, but that didn't seem to do much
* Started increasing kP slowly, which seemed to get the going down positions to be closer while keeping the going up positions the same
* Put kP all the way up to 25, at which point increasing it further seemed to have little effect
* At this point, it seems to work well going up but is still about 0.05 too high going down