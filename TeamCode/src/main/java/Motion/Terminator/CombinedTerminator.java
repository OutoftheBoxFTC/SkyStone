package Motion.Terminator;

import Hardware.SensorData;
import math.Vector3;

public class CombinedTerminator extends Terminator {
    Terminator t1, t2;
    public CombinedTerminator(Vector3 position, Vector3 target, Terminator t1, Terminator t2) {
        super(position, target);
        this.t1 = t1;
        this.t2 = t2;
    }

    public void start(){

    }

    @Override
    public boolean shouldTerminate(SensorData data) {
        return t1.shouldTerminate(data) || t2.shouldTerminate(data);
    }
}
