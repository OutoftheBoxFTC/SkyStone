package Motion;

import com.qualcomm.robotcore.util.RobotLog;

import Hardware.*;
import Odometer.*;
import State.*;
import math.*;

public class CorrectionVector extends DriveState {
    Vector2 target, start;
    Vector3 position;
    Vector4 velocities;
    double targetRot, kp, tolerance, power, firstX, firstY;
    boolean finished, relative = false;
    SimpleOdometer odometer;
    private double kPStrafe = 0.25, kPForward = 0.25, specialFor, specialStr;
    double distance;
    boolean reimannSlowdown = false;
    public CorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, SimpleOdometer odometer){
        super(stateMachine);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
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
    public CorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, boolean slowdown, SimpleOdometer odometer){
        super(stateMachine);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
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
        this.reimannSlowdown = slowdown;
    }
    public CorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, SimpleOdometer odometer, boolean relative){
        super(stateMachine);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
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
        this.relative = relative;
    }
    public CorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, SimpleOdometer odometer, double kpStrafe, double kpForward){
        super(stateMachine);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
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
    }

    @Override
    public Vector4 getWheelVelocities(SensorData data) {
        return velocities;
    }

    public void update(SensorData sensors, HardwareData hardware) {
        RobotLog.ii("Marker", relative ? "True" : "False");
        if(relative){
            target = target.add(new Vector2(position.getA(), position.getB()));
            relative = false;
        }
        double totalDistance = new Vector2(start.getA(), start.getB()).distanceTo(target);
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
        if(Math.abs(x) < 5){
            x *= 0.1;
        }
        if(Math.abs(y) < 5){
            y *= 0.1;
        }
        y *= specialFor;
        x *= specialStr;
        double comb = Math.abs(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        x = x/comb;
        y = y/comb;
        if(reimannSlowdown) {
            power = (-Math.pow(1.075, (((totalDistance-mainr) / totalDistance) * 100) - 100)) + 1;
        }
        x *= power;
        y *= power;
        RobotLog.i("Target: " + target.toString() + " Position: " + position.toString() + " Power: " + new Vector2(x, y).toString());
        RobotLog.i("Mainr: " + mainr + " total distance: " + totalDistance + " power: " + power);
        double rotation = (targetRot - Math.toDegrees(sensors.getGyro()));
        if(rotation > 180){
            rotation = (targetRot - (360 + Math.toDegrees(sensors.getGyro())));
        }else if(rotation < -180){
            rotation = ((360 + targetRot) - Math.toDegrees(sensors.getGyro()));
        }
        rotation *= 0.01;
        velocities.set(MecanumSystem.translate(new Vector3(y, -x, rotation)));
    }

    public void deactivateDriveState(){
        deactivateThis();
    }

    public boolean finished(){
        return Math.abs(new Vector2(position.getA(), position.getB()).distanceTo(target)) < tolerance;
    }
}
