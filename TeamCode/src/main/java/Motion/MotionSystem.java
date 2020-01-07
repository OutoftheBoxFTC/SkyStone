package Motion;

import Odometer.SimpleOdometer;
import State.StateMachine;
import math.Vector2;
import math.Vector3;

public class MotionSystem {
    Vector3 position;
    StateMachine stateMachine;
    SimpleOdometer odometer;
    public MotionSystem(StateMachine stateMachine, SimpleOdometer odometer, Vector3 position){
        this.position = position;
        this.stateMachine = stateMachine;
        this.odometer = odometer;
    }

    public CorrectionVector driveToPoint(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, odometer);
    }

    public CorrectionVector driveToPointSlowdown(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, true, odometer);
    }

    public CorrectionVector driveToPoint(Vector3 target, double power, double forw, double str){
        return new CorrectionVector(stateMachine, position, target, power, odometer, forw, str);
    }

    public CorrectionVector driveForward(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, odometer, true);
    }

    public TurnCorrectionVector turn(Vector3 target, double power) {
        return new TurnCorrectionVector(stateMachine, 0.011, target.getC(), position, power);
    }
}
