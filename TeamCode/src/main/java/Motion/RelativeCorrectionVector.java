package Motion;

import com.qualcomm.robotcore.util.RobotLog;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.StateMachine;
import math.Vector2;
import math.Vector3;
import math.Vector4;

public class RelativeCorrectionVector extends DriveState {
    Vector2 target, start;
    Vector3 position, localTarget;
    Vector4 velocities;
    double targetRot, kp, tolerance, power, firstX, firstY;
    boolean finished;
    SimpleOdometer odometer;
    private double kPStrafe = 0.25, kPForward = 0.25, specialFor, specialStr;
    public RelativeCorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, SimpleOdometer odometer){
        super(stateMachine);
        this.position = position;
        this.localTarget = target;
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector4.ZERO();
        this.tolerance = 1;
        this.kp = 0.2;
        this.odometer = odometer;
        firstX = 0;
        firstY = 0;
        specialFor = 1;
        specialStr = 1;
    }
    public RelativeCorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, SimpleOdometer odometer, double kpStrafe, double kpForward){
        super(stateMachine);
        this.position = position;
        this.localTarget = target;
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector4.ZERO();
        this.tolerance = 1;
        this.kp = 0.2;
        this.odometer = odometer;
        firstX = 0;
        firstY = 0;
        this.specialStr = kpStrafe;
        this.specialFor = kpForward;
    }
    @Override
    public void init(SensorData sensors, HardwareData hardware){
        Vector3 localPos = position.clone();
        Vector3 absoluteTarget = localPos.add(localTarget);
        target = new Vector2(absoluteTarget.getA(), absoluteTarget.getB());
    }

    @Override
    public Vector4 getWheelVelocities(SensorData data) {
        return velocities;
    }

    public void update(SensorData sensors, HardwareData hardware) {
        RobotLog.i("Target: " + target.toString() + " Position: " + position.toString());
        double slope = (start.getB() - target.getB()) / (start.getA() - position.getA());
        double mainr = new Vector2(position.getA(), position.getB()).distanceTo(target);
        double maintheta = (Math.PI/2) - Math.atan2(target.getB() - position.getB(), target.getA() - position.getA());
        double x = mainr * Math.cos(maintheta);
        double y = mainr * Math.sin(maintheta);
        x *= kPStrafe;
        y *= kPForward;
        Vector2 coordinates = new Vector2(x, y);
        double theta = Math.atan2(y,x);
        double r = Math.sqrt((y * y) + (x * x));
        theta += sensors.getGyro();
        x = r * Math.cos(theta);
        y = r * Math.sin(theta);
        if(firstX == 0){
            firstX = Math.abs(x);
            firstY = Math.abs(y);
            firstX = Math.max(firstX, 1);
            firstY = Math.max(firstY, 1);
        }
        x = x / firstX;
        y = y / firstY;
        x *= power;
        y *= power;
        y *= specialFor;
        x *= specialStr;
        double rotation = (targetRot - Math.toDegrees(sensors.getGyro()));
        if(rotation > 180){
            rotation = (targetRot - (360 + Math.toDegrees(sensors.getGyro())));
        }else if(rotation < -180){
            rotation = ((360 + targetRot) - Math.toDegrees(sensors.getGyro()));
        }
        rotation *= 0.01;
        velocities.set(MecanumSystem.translate(new Vector3(x, y, rotation)));
    }

    public void deactivateDriveState(){
        deactivateThis();
    }

    public boolean finished(){
        return Math.abs(new Vector2(position.getA(), position.getB()).distanceTo(target)) < tolerance;
    }
}
