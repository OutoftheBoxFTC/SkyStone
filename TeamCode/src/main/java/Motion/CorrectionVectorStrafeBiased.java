package Motion;

import com.qualcomm.robotcore.util.RobotLog;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.StateMachine;
import State.VelocityDriveState;
import math.Vector2;
import math.Vector3;

public class CorrectionVectorStrafeBiased extends VelocityDriveState {
    private Vector2 target, start;
    private Vector3 position;
    private Vector3 velocities;
    private Vector3 velocity;
    private double targetRot;
    private double power;
    private boolean relative = false;
    private double specialFor;
    private double specialStr;
    private boolean reimannSlowdown = false, linSlowdown = false;
    VelocitySystem system;
    public CorrectionVectorStrafeBiased(StateMachine stateMachine, Vector3 position, Vector3 target, double power, Vector3 velocity){
        super(stateMachine);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        specialFor = 1;
        specialStr = 1;
        this.velocity = velocity;
    }
    public CorrectionVectorStrafeBiased(StateMachine stateMachine, Vector3 position, Vector3 target, double power, Vector3 velocity, boolean slowdown){
        super(stateMachine);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        specialFor = 1;
        specialStr = 1;
        this.velocity = velocity;
        this.linSlowdown = slowdown;
    }
    @Override
    public void init(SensorData sensors, HardwareData hardware){
        system = new VelocitySystem();
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
        if((mainr/totalDistance) < 0.25){
            //system.init(velocity);
        }
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
        specialFor = Math.max(Math.abs(target.getA() - position.getA()) * 2, 1);
        x = x/specialFor;
        y *= specialStr;
        double comb = Math.abs(Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
        x = x/comb;
        y = y/comb;
        double locPower = power;
        if(reimannSlowdown) {
            locPower = Math.abs(Math.max((totalDistance-mainr)/mainr, 0.1));
        }
        if(linSlowdown){
            locPower = power * Math.abs(Math.max((mainr)/totalDistance, 0.3));
        }
        x *= locPower;
        y *= locPower;
        RobotLog.i("Target: " + target.toString() + " Position: " + position.toString() + " Power: " + new Vector2(y, x).toString());
        RobotLog.i("Mainr: " + mainr + " total distance: " + totalDistance + " power: " + power);
        double rotation = (targetRot - Math.toDegrees(sensors.getGyro()));
        if(rotation > 180){
            rotation = (targetRot - (360 + Math.toDegrees(sensors.getGyro())));
        }else if(rotation < -180){
            rotation = ((360 + targetRot) - Math.toDegrees(sensors.getGyro()));
        }
        rotation *= 0.01;
        //velocities.set(system.update(new Vector3(y, -x, rotation), (mainr/totalDistance)));
        velocities.set(new Vector3(y, -x, rotation));
    }

}
