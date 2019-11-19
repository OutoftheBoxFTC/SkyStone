package motion.terminator;

import hardware.ReadData;
import math.MathUtil;
import math.Vector2;
import math.Vector3;

public class PositionTerminator extends Terminator {
    private Vector3 position, target;

    private SensitivityTerminator distanceTerminator, rotationTerminator;

    public PositionTerminator(Vector3 position, Vector3 target, double dTolerance, double rTolerance, double time){
        this.position = position;
        this.target = target;
        distanceTerminator = new SensitivityTerminator(time, dTolerance, true);
        rotationTerminator = new SensitivityTerminator(time, rTolerance, true);
    }

    public SensitivityTerminator getRotationTerminator() {
        return rotationTerminator;
    }

    public SensitivityTerminator getDistanceTerminator() {
        return distanceTerminator;
    }

    @Override
    public boolean shouldTerminate(ReadData data) {
        double distanceErr = new Vector2(position).distanceTo(new Vector2(target));
        double rotationErr = MathUtil.angleDelta(position.getC(), target.getC());
        return distanceTerminator.shouldTerminate(distanceErr, data.getHub1BulkTime())&&rotationTerminator.shouldTerminate(rotationErr, data.getHub1BulkTime());
    }
}