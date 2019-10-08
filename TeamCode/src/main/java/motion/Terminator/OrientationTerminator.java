package motion.Terminator;

import hardware.ReadData;
import math.Vector2;
import math.Vector3;

public class OrientationTerminator extends Terminator {
    double movementTolerance, rotationTolerance;
    public OrientationTerminator(Vector3 position, Vector3 target, double movementTolerance, double rotationTolerance) {
        super(position, target);
        this.movementTolerance = movementTolerance;
        this.rotationTolerance = rotationTolerance;
    }

    @Override
    public boolean shouldTerminate(ReadData data) {
        Vector2 coordinates = new Vector2(position.getA(), position.getB());
        Vector2 targetCoords = new Vector2(position.getA(), position.getB());
        double distance = coordinates.distanceTo(targetCoords);
        double rotation = Math.abs(position.getC() - target.getC());
        return (distance <= movementTolerance && rotation <= rotationTolerance);
    }
}
