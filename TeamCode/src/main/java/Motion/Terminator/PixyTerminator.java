package Motion.Terminator;

import Hardware.SensorData;
import math.Vector3;

public class PixyTerminator extends Terminator {
    public PixyTerminator(Vector3 position, Vector3 target) {
        super(position, target);
    }

    @Override
    public boolean shouldTerminate(SensorData data) {
        int colour = data.getPixy()[data.getPixy().length-2] & 0xFF;
        return Math.abs(colour) < 160;
    }
}
