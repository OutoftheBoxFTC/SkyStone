package Motion.Terminator;

import Hardware.SensorData;
import math.Vector3;

public class NullTerminator extends Terminator {
    public NullTerminator() {
        super(Vector3.ZERO(), Vector3.ZERO());
    }

    @Override
    public boolean shouldTerminate(SensorData data) {
        return false;
    }
}
