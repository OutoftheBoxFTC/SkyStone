package motion;

import drivetrain.RobotDrive;
import math.Vector3;
import math.Vector4;
import state.DriveState;
import state.StateMachine;

public abstract class VelocityDriveState extends DriveState {
    private RobotDrive robotDrive;
    public VelocityDriveState(StateMachine stateMachine, RobotDrive robotDrive) {
        super(stateMachine);
        this.robotDrive = robotDrive;
    }

    @Override
    public Vector4 getWheelVelocities() {
        Vector3 robotVelocity = getRobotVelocity();
        Vector4 wheels = robotDrive.getWheelVelocities(robotVelocity);
        double downScale = Math.max(wheels.getA(), Math.max(wheels.getB(), Math.max(wheels.getC(), wheels.getD())));
        downScale = Math.max(1, downScale);
        wheels = wheels.scale(1/downScale);
        return wheels;
    }

    protected abstract Vector3 getRobotVelocity();
}