package Motion.Terminator;

import Hardware.SensorData;
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
    public boolean shouldTerminate(SensorData data) {
        Vector2 coordinates = new Vector2(position.getA(), position.getB());
        Vector2 targetCoords = new Vector2(target.getA(), target.getB());
        double distance = coordinates.distanceTo(targetCoords);
        double rotation = Math.abs(position.getC() - (target.getC()>0 ? target.getC() : (360-Math.abs(target.getC()))));
        return (distance <= movementTolerance);
    }

    public static boolean shouldTerminatePosition(Vector3 position, Vector3 target, double movementTolerance){
        Vector2 coordinates = new Vector2(position.getA(), position.getB());
        Vector2 targetCoords = new Vector2(target.getA(), target.getB());
        double distance = coordinates.distanceTo(targetCoords);
        return (distance <= movementTolerance);
    }

    public static boolean shouldTerminateRotation(double angle, double target, double tolerance){
        return Math.abs((angle - (target < 0 ? 360 + target : target))) < tolerance;
    }

    public boolean shouldTerminateRotation(){
        return Math.abs(position.getC() - target.getC()) < rotationTolerance;
    }
}
