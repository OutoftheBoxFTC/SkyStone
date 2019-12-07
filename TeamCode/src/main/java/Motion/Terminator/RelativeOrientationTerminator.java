package Motion.Terminator;

import Hardware.SensorData;
import math.Vector3;

public class RelativeOrientationTerminator extends Terminator {
    Vector3 position, target;
    double tolerance;
    public RelativeOrientationTerminator(Vector3 position, Vector3 target, double tolerance) {
        super(position, target);
        this.position = position;
        this.target = target;
        this.tolerance = tolerance;
    }
    public void start(){
        target = target.add(new Vector3(position.getA(), position.getB(), 0));
    }
    @Override
    public boolean shouldTerminate(SensorData data) {
        return OrientationTerminator.shouldTerminatePosition(position, target, tolerance);
    }
}
