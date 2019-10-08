package state.motion;

import com.qualcomm.robotcore.util.RobotLog;

import Odometer.SimpleOdometer;
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
    double targetRot, offset, kp, tolerance, AoA, power;
    Terminator terminator;
    SimpleOdometer odometer;

    public CorrectionVector(StateMachine stateMachine, RobotDrive robotDrive, Vector3 position, Vector3 target, double AoA, Terminator terminator, double power, SimpleOdometer odometer){
        super(stateMachine, robotDrive);
        this.position = position;
        this.target = new Vector2(target.getA(), target.getB());
        this.AoA = AoA + 90;
        this.terminator = terminator;
        targetRot = new Vector2(position.getA(), position.getB()).angleTo(new Vector2(target.getA(), target.getB()));
        this.power = power;
        this.start = new Vector2(position.getA(), position.getB());
        this.velocities = Vector3.ZERO();
        this.tolerance = 1;
        this.kp = 0.2;
        this.odometer = odometer;
    }
    @Override
    public void init(ReadData data){
        odometer.start(data);
        offset = (data.getLeft() + data.getRight())/2;
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocities;
    }

    public void update(ReadData data) {
        if((!terminator.shouldTerminate(data)) && !finished()) {
            double slope = (start.getB() - target.getB()) / (start.getA() - position.getA());
            //Matrix22 formulaMatrix = new Matrix22(slope, -1, (-1 / slope), -1).inverse();
            //Vector2 solutionAnswers = new Vector2(-target.getB() + (slope * target.getA()), -position.getB() + (-1 / slope * position.getA()));
            //Vector2 solutions = formulaMatrix.transform(solutionAnswers);
            //double x = new Vector2(position.getA(), position.getB()).distanceTo(solutions) * (position.getA() - solutions.getA() / Math.abs(position.getA() - solutions.getA()));
            //double y = new Vector2(position.getA(), position.getB()).distanceTo(target);
            //double x = position.getA() - target.getA();
            double mainr = new Vector2(position.getA(), position.getB()).distanceTo(target);
            double maintheta = (Math.PI/2) - Math.atan2(target.getB(), target.getA());
            double x = mainr * Math.cos(maintheta);
            double y = mainr * Math.sin(maintheta);
            if(x <= (5)){
                velocities = Vector3.ZERO();
                deactivateDriveState();
            }else {
                Vector2 coordinates = new Vector2(x, y);
                double theta = Math.atan2(y,x);
                double r = Math.sqrt((y * y) + (x * x));
                theta += Math.toRadians(AoA);
                x = r * Math.cos(theta);
                y = r * Math.sin(theta);
                x = Math.min(x, power);
                x = Math.max(x, -power);
                y = Math.min(y, power);
                y = Math.max(y, -power);
                velocities.set(new Vector3(x, y, 0));
            }
        }else{
            velocities = Vector3.ZERO();
            //deactivateDriveState();
        }
    }

    public void deactivateDriveState(){
        deactivateThis();
    }

    public boolean finished(){
        return Math.abs(new Vector2(position.getA(), position.getB()).distanceTo(target)) < tolerance;
    }
}
