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

    public RelativeCorrectionVector driveForward(Vector3 target, double power){
        return new RelativeCorrectionVector(stateMachine, position, target, power, odometer);
    }
}
