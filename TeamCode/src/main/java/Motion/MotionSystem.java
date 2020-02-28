package Motion;

import State.StateMachine;
import math.Vector3;

public class MotionSystem {
    private Vector3 position, velocity;
    private StateMachine stateMachine;
    private VelocitySystem system;
    public MotionSystem(StateMachine stateMachine, Vector3 position, Vector3 velocity){
        this.position = position;
        this.stateMachine = stateMachine;
        this.system = system;
    }

    public CorrectionVector driveToPoint(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, velocity);
    }

    public CorrectionVector driveToPointLinSlowdown(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, velocity, true);
    }

    public CorrectionVector driveToPointLinSlowdown(Vector3 target, double power, double minPower){
        return new CorrectionVector(stateMachine, position, target, power, velocity, true, minPower);
    }

    public CorrectionVector driveToPointSlowdown(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, velocity);
    }

    public CorrectionVector driveToPoint(Vector3 target, double power, double forw, double str){
        return new CorrectionVector(stateMachine, position, target, power, forw, str, velocity);
    }

    public CorrectionVector driveForward(Vector3 target, double power){
        return new CorrectionVector(stateMachine, position, target, power, false, velocity, true);
    }

    public TurnCorrectionVector turn(Vector3 target, double power) {
        return new TurnCorrectionVector(stateMachine, 0.011, target.getC(), position, power);
    }
}
