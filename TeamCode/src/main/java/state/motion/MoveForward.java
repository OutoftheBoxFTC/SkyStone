package state.motion;

import drivetrain.RobotDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import motion.VelocityDriveState;
import state.StateMachine;

public class MoveForward extends VelocityDriveState {
    double forward, current, speed, translationFactor, target;
    Vector3 velocities = Vector3.ZERO();
    String nextState;
    Hardware robot;
    public MoveForward(StateMachine stateMachine, RobotDrive robotDrive, ReadData data, double speed, String nextState, double translationFactor, double target, Hardware robot) {
        super(stateMachine, robotDrive);
        forward = (data.getLeft() + data.getRight())/2;
        current = forward;
        this.speed = speed;
        this.nextState = nextState;
        this.translationFactor = translationFactor;
        this.target = target;
        this.robot = robot;
    }
    @Override
    public void init(ReadData data){
        forward = (data.getLeft() + data.getRight())/2;
        current = forward;
    }
    @Override
    public void update(ReadData data){
        velocities = new Vector3(0, -speed, 0);
        current = (data.getLeft() + data.getRight())/2;
        if(Math.abs((current * translationFactor) - (forward * translationFactor)) > Math.abs(target)){
            stateMachine.setActiveDriveState(nextState);
            if(target < 0) {
                robot.intake(0);
            }else{
                robot.intake(1);
            }
            deactivateThis();
        }
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocities;
    }
}
