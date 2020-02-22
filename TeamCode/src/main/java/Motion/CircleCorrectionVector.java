package Motion;

import com.qualcomm.robotcore.util.RobotLog;

import Debug.Connector;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.StateMachine;
import State.VelocityDriveState;
import math.Vector2;
import math.Vector3;
import math.Vector4;

public class CircleCorrectionVector extends VelocityDriveState {
    private Vector3 target, splinePoint, velocities, position;
    private Vector4 lineCoords;
    private double power, radius;
    public CircleCorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, Vector3 splinePoint, double power, double radius) {
        super(stateMachine);
        this.target = target;
        this.power = power;
        this.radius = radius;
        this.splinePoint = splinePoint;
        this.position = position;
        velocities = Vector3.ZERO();
    }

    public CircleCorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, Vector3 splinePoint, double power){
        this(stateMachine, position, target, splinePoint, power, 10);
    }

    @Override
    public Vector3 getVelocities() {
        return velocities;
    }

    @Override
    public void update(SensorData sensors, HardwareData hardware) {
        Vector2 localTarget = new Vector2(splinePoint.getA(), splinePoint.getB());
        Vector2 posCoord = position.getVector2();
        Vector4 intersections = findLineCircleIntersections(position.getA(), position.getB(), radius, splinePoint.getVector2(), target.getVector2());
        RobotLog.i(intersections.toString());
        if(!Double.isNaN(intersections.getA()) && !Double.isNaN(intersections.getC())){
            Vector2 coordsA = new Vector2(intersections.getA(), intersections.getB());
            Vector2 coordsB = new Vector2(intersections.getC(), intersections.getD());
            localTarget.set(((coordsA.distanceTo(target.getVector2()) < coordsB.distanceTo(target.getVector2()))) ? coordsA : coordsB);
        }else if(Double.isNaN(intersections.getA())){
            localTarget = new Vector2(intersections.getC(), intersections.getD());
        }else if(Double.isNaN(intersections.getC())){
            localTarget = new Vector2(intersections.getA(), intersections.getB());
        }
        RobotLog.i(localTarget.toString());
        Vector2 powers = localTarget.add(posCoord.scale(-1));
        double r = Math.sqrt((powers.getA() * powers.getA()) + (powers.getB() * powers.getB()));
        double theta = Math.atan2(powers.getB(), powers.getA()) + sensors.getGyro();
        powers.set(new Vector2(r * (Math.cos(theta)), r * (Math.sin(theta))));
        double scaleVal = Math.sqrt((powers.getA() * powers.getA()) + (powers.getB() * powers.getB()));
        double targetRot = target.getC();
        powers.set(powers.scale((1/scaleVal)));
        //Connector.getInstance().addTelemetry("Powers", powers.toString());
        //Connector.getInstance().addTelemetry("Rotation", targetRot);
        powers.set(powers.scale(power));
        double rotation = (targetRot - Math.toDegrees(sensors.getGyro()));
        if(rotation > 180){
            rotation = (targetRot - (360 + Math.toDegrees(sensors.getGyro())));
        }else if(rotation < -180){
            rotation = ((360 + targetRot) - Math.toDegrees(sensors.getGyro()));
        }
        RobotLog.i(powers.toString());
        RobotLog.i(String.valueOf(rotation));
        rotation = rotation / 360;
        rotation *= power;
        velocities.set(new Vector3(powers.getA(), -powers.getB(), 0));
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
            return new Vector4(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        }
        else if (det == 0)
        {
            // One solution.
            t = -B / (2 * A);
            Vector2 intersection1 =
                    new Vector2(point1.getA() + t * dx, point1.getB() + t * dy);
            Vector2 intersection2 = new Vector2(Double.NaN, Double.NaN);
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
