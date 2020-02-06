package Motion.Terminator;

import HardwareSystems.SensorData;
import math.Vector3;

public class TimerTerminator extends Terminator{
    long timer;

    public TimerTerminator(Vector3 position, Vector3 target, long timer) {
        super(position, target);
        this.timer = System.currentTimeMillis() + timer;
    }

    @Override
    public boolean shouldTerminate(SensorData data) {
        return System.currentTimeMillis() >= timer;
    }
}
