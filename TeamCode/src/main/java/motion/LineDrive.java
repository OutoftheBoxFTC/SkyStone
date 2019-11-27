package motion;

import drivetrain.RobotDrive;
import math.Vector3;
import state.StateMachine;
import state.motion.VelocityDriveState;

public class LineDrive extends VelocityDriveState {

    public LineDrive(StateMachine stateMachine, RobotDrive robotDrive) {
        super(stateMachine, robotDrive);
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return null;
    }
}
