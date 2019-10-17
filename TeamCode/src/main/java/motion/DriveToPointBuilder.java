package motion;

import Odometer.SimpleOdometer;
import drivetrain.RobotDrive;
import math.Vector3;
import motion.Terminator.OrientationTerminator;
import motion.Terminator.Terminator;
import state.DriveState;
import state.StateMachine;
import state.motion.CorrectionVector;
import state.motion.TurnCorrectionVector;

public class DriveToPointBuilder {
    protected final static double FORWARD = 0, BACKWARDS = 0, RIGHT = -90, LEFT = 90;
    StateMachine machine;
    RobotDrive drive;
    Vector3 position, velocity;
    SimpleOdometer odometer;
    public DriveToPointBuilder(StateMachine stateMachine, RobotDrive robotDrive, Vector3 position, Vector3 velocity, SimpleOdometer odometer){
        this.machine = stateMachine;
        this.drive = robotDrive;
        this.position = position;
        this.velocity = velocity;
        this.odometer = odometer;
    }
    public DriveState create(Vector3 target, double AoA, Terminator terminator, double power, String nextState){
        return new CorrectionVector(machine, drive, position, target, AoA, terminator, power, odometer, nextState);
    }
    public DriveState turn(double kP, double angle, String nextState, OrientationTerminator terminator){
        return new TurnCorrectionVector(machine, drive, kP, angle, 0, position, nextState, terminator);
    }
}
