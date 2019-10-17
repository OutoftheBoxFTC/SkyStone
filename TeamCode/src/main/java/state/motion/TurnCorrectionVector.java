package state.motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.Vector3;
import motion.Terminator.OrientationTerminator;
import motion.Terminator.Terminator;
import motion.VelocityDriveState;
import state.Orientation;
import state.StateMachine;

public class TurnCorrectionVector extends VelocityDriveState {
    double targetAngle, offset, kp, correction, tolerance;
    boolean finished = false;
    String nextState;
    Vector3 position;
    OrientationTerminator terminator;
    public TurnCorrectionVector(StateMachine stateMachine, RobotDrive drive, double kp, double angle, double tolerance, Vector3 position, String nextState, OrientationTerminator terminator){
        super(stateMachine, drive);
        this.kp = kp;
        targetAngle = angle;
        finished = false;
        this.tolerance = tolerance;
        this.position = position;
        this.nextState = nextState;
        this.terminator = terminator;
    }
    @Override
    public void update(ReadData data) {
        correction = (targetAngle - (Math.toDegrees(data.getGyro())));
        if(correction > 180){
            correction = (targetAngle - (360 + Math.toDegrees(data.getGyro())));
        }else if(correction < -180){
            correction = ((360 + targetAngle) - Math.toDegrees(data.getGyro()));
        }
        correction *= kp;
        if(terminator.shouldTerminateRotation()){
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
