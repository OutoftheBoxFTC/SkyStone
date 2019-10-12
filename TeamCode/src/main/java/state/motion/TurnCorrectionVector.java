package state.motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.Vector3;
import motion.VelocityDriveState;
import state.StateMachine;

public class TurnCorrectionVector extends VelocityDriveState {
    double targetAngle, offset, kp, correction, tolerance;
    boolean finished = false;
    Vector3 position;
    public TurnCorrectionVector(StateMachine stateMachine, RobotDrive drive){
        super(stateMachine, drive);
    }
    public void start(double kp, double angle, double tolerance, Vector3 position) {
        this.kp = kp;
        targetAngle = angle;
        finished = false;
        this.tolerance = tolerance;
        this.position = position;
    }
    @Override
    public void update(ReadData data) {
        correction = kp * (targetAngle - (data.getGyro() - offset));
        finished = Math.abs(targetAngle - (data.getGyro() - offset)) < tolerance;
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return new Vector3(0, 0, correction);
    }

    public boolean finished(){
        return finished;
    }
}
