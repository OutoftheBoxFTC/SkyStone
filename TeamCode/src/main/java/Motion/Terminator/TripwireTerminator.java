package Motion.Terminator;

import HardwareSystems.SensorData;
import math.Vector3;

public class TripwireTerminator extends Terminator {

    public TripwireTerminator(Vector3 position, Vector3 target) {
        super(position, target);
    }

    @Override
    public boolean shouldTerminate(SensorData data) {
        return data.getIntakeTripwire() < 1;
    }
}
