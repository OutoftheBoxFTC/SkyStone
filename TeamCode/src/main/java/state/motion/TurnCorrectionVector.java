package state.motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.Vector3;
import motion.VelocityDriveState;
import state.StateMachine;

public class TurnCorrectionVector extends VelocityDriveState {
    double targetAngle, offset, kp, correction, tolerance;
    boolean finished = false;
    String nextState;
    Vector3 position;
    public TurnCorrectionVector(StateMachine stateMachine, RobotDrive drive, double kp, double angle, double tolerance, Vector3 position, String nextState){
        super(stateMachine, drive);
        this.kp = kp;
        targetAngle = angle;
        finished = false;
        this.tolerance = tolerance;
        this.position = position;
        this.nextState = nextState;
    }
    @Override
    public void update(ReadData data) {
        correction = kp * (targetAngle - (Math.toDegrees(data.getGyro())));
        finished = Math.abs(targetAngle - (Math.toDegrees(data.getGyro()))) < tolerance;
        if(finished){
            deactivateThis();
            stateMachine.setActiveDriveState(nextState);
        }
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return new Vector3(0, 0, correction);
    }

    public boolean finished(){
        return finished;
    }
}
