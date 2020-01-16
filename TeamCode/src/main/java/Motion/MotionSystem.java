package Motion;

import State.StateMachine;
import math.Vector3;

public class MotionSystem {
    private Vector3 position;
    private StateMachine stateMachine;
    private VelocitySystem system;
    public MotionSystem(StateMachine stateMachine, Vector3 position, VelocitySystem system){
        this.position = position;
        this.stateMachine = stateMachine;
        this.system = system;
    }

    public CorrectionVector driveToPoint(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, system);
    }

    public CorrectionVector driveToPointSlowdown(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, system);
    }

    public CorrectionVector driveToPoint(Vector3 target, double power, double forw, double str){
        return new CorrectionVector(stateMachine, position, target, power, forw, str, system);
    }

    public CorrectionVector driveForward(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, true, system);
    }

    public TurnCorrectionVector turn(Vector3 target, double power) {
        return new TurnCorrectionVector(stateMachine, 0.011, target.getC(), position, power);
    }
}
