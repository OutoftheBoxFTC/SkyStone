package Motion;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.StateMachine;
import State.VelocityDriveState;
import math.Vector2;
import math.Vector3;
import math.Vector4;

public class CurveCorrectionSystem extends VelocityDriveState {
    Vector3 velocities, position;
    double power, radius, slowdownDistance, minPower, maxDistance, angle, startTurnDistance, initialAngle;
    boolean optimiseAngle;
    ArrayList<Vector2> splinePoints;
    public CurveCorrectionSystem(StateMachine stateMachine, Vector3 position, ArrayList<Vector2> splinePoints, double power, double radius, double slowdownDistance, double minPower, double angle, double startTurnDistance, double initialAngle, boolean optimiseAngle) {
        super(stateMachine);
        this.position = position;
        this.splinePoints = splinePoints;
        this.power = power;
        this.radius = radius;
        this.slowdownDistance = slowdownDistance;
        this.minPower = minPower;
        this.maxDistance = 0;
        this.angle = angle;
        velocities = Vector3.ZERO();
        this.startTurnDistance = startTurnDistance;
        this.initialAngle = initialAngle;
        this.optimiseAngle = optimiseAngle;
    }

    @Override
    public Vector3 getVelocities() {
        return velocities;
    }

    @Override
    public void update(SensorData sensors, HardwareData hardware) {
        ArrayList<Vector2> intersections = new ArrayList<>();
        for(int i = 0; i < splinePoints.size()-1; i ++){
            Vector4 intersecs = findLineCircleIntersections(position.getA(), position.getB(), radius, splinePoints.get(i), splinePoints.get(i+1));
            intersections.add(new Vector2(intersecs.getA(), intersecs.getB()));
            intersections.add(new Vector2(intersecs.getC(), intersecs.getD()));
        }
        Vector2 closestPoint = new Vector2(0, 0);
        double minDistance = 1000000;
        for(Vector2 v : intersections){
            if(v.distanceTo(splinePoints.get(splinePoints.size()-1)) < minDistance){
                minDistance = v.distanceTo(splinePoints.get(splinePoints.size()-1));
                closestPoint.set(v);
            }
        }
        double localPower = power;
        if(position.getVector2().distanceTo(splinePoints.get(splinePoints.size()-1)) < slowdownDistance){
            if(maxDistance == 0){
                maxDistance = position.getVector2().distanceTo(splinePoints.get(splinePoints.size()-1));
            }
            localPower = Range.clip(localPower, minPower, position.getVector2().distanceTo(splinePoints.get(splinePoints.size()-1))/maxDistance);
        }
        int count = 0;
        for(Vector2 i : intersections){
            if(i.getA() == Double.POSITIVE_INFINITY){

            }else{
                count ++;
            }
        }
        double currRadius = radius;
        while(count == 0){
            for(int i = 0; i < splinePoints.size()-1; i ++){
                Vector4 intersecs = findLineCircleIntersections(position.getA(), position.getB(), currRadius, splinePoints.get(i), splinePoints.get(i+1));
                intersections.add(new Vector2(intersecs.getA(), intersecs.getB()));
                intersections.add(new Vector2(intersecs.getC(), intersecs.getD()));
            }
            count = 0;
            for(Vector2 i : intersections){
                if(i.getA() == Double.POSITIVE_INFINITY){

                }else{
                    count ++;
                }
            }
            currRadius += 0.25;
        }
        closestPoint = new Vector2(0, 0);
        minDistance = 1000000;
        for(Vector2 v : intersections){
            if(v.distanceTo(splinePoints.get(splinePoints.size()-1)) < minDistance){
                minDistance = v.distanceTo(splinePoints.get(splinePoints.size()-1));
                closestPoint.set(v);
            }
        }
        if(count > 0){
            if((position.getVector2().distanceTo(splinePoints.get(splinePoints.size()-1)) < startTurnDistance)) {
                moveToPoint(closestPoint.toVector3(angle), localPower);
            }else{
                moveToPoint(closestPoint.toVector3(initialAngle), localPower);
            }
        }else{
            moveToPoint(splinePoints.get(0).toVector3(angle), power);
        }
    }

    private void moveToPoint(Vector3 target, double power){
        double r = new Vector2(target.getA(), target.getB()).distanceTo(new Vector2(position.getA(), position.getB()));
        double angle = Math.atan2(target.getB() - position.getB(), target.getA() - position.getA());
        double targetRot = target.getC();
        if(optimiseAngle){
            targetRot = Math.toDegrees(angle) - 270;
        }
        angle -= Math.toRadians(position.getC());
        Vector2 relativePowers = new Vector2(r * Math.cos(angle), r * Math.sin(angle));
        double magnitude = Math.abs(relativePowers.getA()) + Math.abs(relativePowers.getB());
        double rotation = (targetRot - position.getC());
        if(rotation > 180){
            rotation = (targetRot - (360 + position.getC()));
        }else if(rotation < -180){
            rotation = ((360 + targetRot) - position.getC());
        }
        rotation = Range.clip(rotation / 30, -1, 1);
        if(Math.abs(rotation) < 0.1){
            rotation = (Math.abs(rotation)/rotation) * 0.1;
        }
        Vector2 motors = new Vector2(relativePowers.getA(), relativePowers.getB());
        if(motors.getA() > 0.1){
            motors.setA(relativePowers.getA()/magnitude);
        }else{
            motors.setA(0);
        }
        if(motors.getB() > 0.1){
            motors.setB(relativePowers.getB()/magnitude);
        }else{
            motors.setB(0);
        }
        velocities.set(power * (relativePowers.getA() / magnitude), -power * (relativePowers.getB()/ magnitude), rotation * power);
    }

    private Vector4 findLineCircleIntersections(
            double cx, double cy, double radius,
            Vector2 point1, Vector2 point2)
    {
        double dx, dy, A, B, C, det, t;

        dx = point2.getA() - point1.getA();
        dy = point2.getB() - point1.getB();

        A = dx * dx + dy * dy;
        B = 2 * (dx * (point1.getA() - cx) + dy * (point1.getB() - cy));
        C = (point1.getA() - cx) * (point1.getA() - cx) +
                (point1.getB() - cy) * (point1.getB() - cy) -
                radius * radius;

        det = B * B - 4 * A * C;
        if ((A <= 0.0000001) || (det < 0))
        {
            // No real solutions.
            return new Vector4(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        else if (det == 0)
        {
            // One solution.
            t = -B / (2 * A);
            Vector2 intersection1 =
                    new Vector2(point1.getA() + t * dx, point1.getB() + t * dy);
            Vector2 intersection2 = new Vector2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
            return new Vector4(intersection1.getA(), intersection1.getB(), intersection2.getA(), intersection2.getB());
        }
        else
        {
            // Two solutions.
            t = (float)((-B + Math.sqrt(det)) / (2 * A));
            Vector2 intersection1 =
                    new Vector2(point1.getA() + t * dx, point1.getB() + t * dy);
            t = (float)((-B - Math.sqrt(det)) / (2 * A));
            Vector2 intersection2 =
                    new Vector2(point1.getA() + t * dx, point1.getB() + t * dy);
            return new Vector4(intersection1.getA(), intersection1.getB(), intersection2.getA(), intersection2.getB());
        }
    }
}
