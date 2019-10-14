package motion;

import drivetrain.RobotDrive;
import math.Vector3;
import motion.Terminator.Terminator;
import state.DriveState;
import state.StateMachine;
import state.motion.CorrectionVector;

public class DriveToPointBuilder {
    protected final static double FORWARD = 0, BACKWARDS = 0, RIGHT = -90, LEFT = 90;
    StateMachine machine;
    RobotDrive drive;
    public DriveToPointBuilder(StateMachine stateMachine, RobotDrive robotDrive){
        this.machine = stateMachine;
        this.drive = robotDrive;
    }
    public  DriveState create(Vector3 position, Vector3 velocity, Vector3 target, double AoA, Terminator terminator, double power){
        return new CorrectionVector(machine, drive, position, target, AoA, terminator, power, null, null);
    }
}
