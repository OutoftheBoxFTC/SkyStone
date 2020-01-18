package Motion;

import com.qualcomm.robotcore.util.RobotLog;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.StateMachine;
import State.VelocityDriveState;
import math.Vector2;
import math.Vector3;

public class CorrectionVector extends VelocityDriveState {
    private Vector2 target, start;
    private Vector3 position;
    private Vector3 velocities;
    private double targetRot;
    private double power;
    private boolean relative = false;
    private double specialFor;
    private double specialStr;
    private boolean reimannSlowdown = false;
    CorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, VelocitySystem system){
        super(stateMachine, system);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        specialFor = 1;
        specialStr = 1;
    }
    CorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, boolean slowdown, VelocitySystem system){
        super(stateMachine, system);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        specialFor = 1;
        specialStr = 1;
        this.reimannSlowdown = slowdown;
    }
    CorrectionVector(StateMachine stateMachine, Vector3 position, Vector3 target, double power, double kpStrafe, double kpForward, VelocitySystem system){
        super(stateMachine, system);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        this.specialStr = kpStrafe;
        this.specialFor = kpForward;
    }
    @Override
    public void init(SensorData sensors, HardwareData hardware){
    }

    @Override
    public Vector3 getVelocities() {
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
        double kPStrafe = 0.25;
        x *= kPStrafe;
        double kPForward = 0.25;
        y *= kPForward;
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
        velocities.set((new Vector3(y, -x, rotation)));
    }

}
