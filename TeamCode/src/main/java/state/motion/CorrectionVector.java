package state.motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import motion.VelocityDriveState;
import state.StateMachine;

public class CorrectionVector extends VelocityDriveState {
    MOVEMENT_TYPE movement_type;
    Vector2 start, target;
    Vector3 velocities, position;
    double targetRot, offset, kp, tolerance;

    public CorrectionVector(StateMachine stateMachine, RobotDrive robotDrive){
        super(stateMachine, robotDrive);
    }

    public void start(Vector2 target, MOVEMENT_TYPE movement_type, Vector3 position, double wantedRot, double kp, double tolerance) {
        this.target = target;
        start = new Vector2(position.getA(), position.getB());
        this.movement_type = movement_type;
        this.position = position;
        targetRot = wantedRot;
        offset = position.getC();
        this.kp = kp;
        this.tolerance = tolerance;
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocities;
    }

    public void update(ReadData data) {
        double slope = (start.getB() - target.getB())/(start.getA() - position.getA());
        Matrix22 formulaMatrix = new Matrix22(slope, -1, (-1/slope), -1).inverse();
        Vector2 solutionAnswers = new Vector2(-target.getB() + (slope * target.getA()), -position.getB() + (-1/slope * position.getA()));
        Vector2 solutions = formulaMatrix.transform(solutionAnswers);
        double x = new Vector2(position.getA(), position.getB()).distanceTo(solutions);
        double y = new Vector2(position.getA(), position.getB()).distanceTo(target);
        if(movement_type == MOVEMENT_TYPE.FORWARD) {
            velocities = new Vector3(x, y, (targetRot - (position.getC()-offset)) * kp);
        }else{
            Vector2 coordinates = new Vector2(x, y);
            double theta = Math.atan(y/x);
            double r = Math.sqrt((y * y) + (x * x));
            theta += (Math.toRadians(90)) * movement_type.getConstant();
            x = r * Math.cos(theta);
            y = r * Math.sin(theta);
            velocities = new Vector3(x, y, ((targetRot - (position.getC()-offset)) * kp) + (movement_type == MOVEMENT_TYPE.STRAFE ? 90 : -90));
        }
    }

    public void deactivateDriveState(){
        deactivateThis();
    }

    public boolean finished(){
        return Math.abs(new Vector2(position.getA(), position.getB()).distanceTo(target)) < tolerance;
    }

    public static enum MOVEMENT_TYPE{
        FORWARD(0),
        STRAFE(1),
        INVERSE_STRAFE(-1);
        private final int constant;
        MOVEMENT_TYPE(int constant) {
            this.constant = constant;
        }

        public int getConstant() {
            return constant;
        }
    }
}
