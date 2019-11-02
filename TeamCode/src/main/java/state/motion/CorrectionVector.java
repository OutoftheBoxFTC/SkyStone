package state.motion;

import com.qualcomm.robotcore.util.RobotLog;

import Odometer.SimpleOdometer;
import debug.SmartTelemetry;
import drivetrain.RobotDrive;
import hardware.ReadData;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import motion.Terminator.Terminator;
import motion.VelocityDriveState;
import state.StateMachine;

public class CorrectionVector extends VelocityDriveState {
    Vector2 target;
    Vector2 start;
    Vector3 velocities, position;
    double targetRot, kp, tolerance, AoA, power, firstX, firstY;
    boolean finished;
    Terminator terminator;
    SimpleOdometer odometer;
    private double kPStrafe = 0.25, kPForward = 0.25, specialFor, specialStr;
    String nextstate;
    public CorrectionVector(StateMachine stateMachine, RobotDrive robotDrive, Vector3 position, Vector3 target, double AoA, Terminator terminator, double power, SimpleOdometer odometer, String nextState){
        super(stateMachine, robotDrive);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        this.AoA = AoA - 90;
        this.terminator = terminator;
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        this.tolerance = 1;
        this.kp = 0.2;
        this.odometer = odometer;
        this.nextstate = nextState;
        firstX = 0;
        firstY = 0;
        specialFor = 1;
        specialStr = 1;
    }
    public CorrectionVector(StateMachine stateMachine, RobotDrive robotDrive, Vector3 position, Vector3 target, double AoA, Terminator terminator, double power, SimpleOdometer odometer, String nextState, double kpStrafe, double kpForward){
        super(stateMachine, robotDrive);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        this.AoA = AoA - 90;
        this.terminator = terminator;
        targetRot = target.getC();
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        this.tolerance = 1;
        this.kp = 0.2;
        this.odometer = odometer;
        this.nextstate = nextState;
        firstX = 0;
        firstY = 0;
        this.specialStr = kpStrafe;
        this.specialFor = kpForward;
    }
    @Override
    public void init(ReadData data){

    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocities;
    }

    public void update(ReadData data) {
        if((!terminator.shouldTerminate(data)) && !finished) {
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
            theta += Math.toRadians(AoA);
            theta += data.getGyro();
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
            x = Math.min(x, power);
            x = Math.max(x, -power);
            y = Math.min(y, power);
            y = Math.max(y, -power);
            y *= specialFor;
            x *= specialStr;
            double rotation = (targetRot - Math.toDegrees(data.getGyro()));
            if(rotation > 180){
                rotation = (targetRot - (360 + Math.toDegrees(data.getGyro())));
            }else if(rotation < -180){
                rotation = ((360 + targetRot) - Math.toDegrees(data.getGyro()));
            }
            rotation *= 0.01;
            velocities.set(new Vector3(x, y, rotation));
        }else{
            velocities = Vector3.ZERO();
            deactivateDriveState();
        }
    }

    public void deactivateDriveState(){
        stateMachine.setActiveDriveState(nextstate);
        deactivateThis();
    }

    public boolean finished(){
        return Math.abs(new Vector2(position.getA(), position.getB()).distanceTo(target)) < tolerance;
    }
}
