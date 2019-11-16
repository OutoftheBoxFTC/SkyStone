package motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.MathUtil;
import math.Vector2;
import math.Vector3;
import motion.terminator.Terminator;
import state.StateMachine;

public class LineDrive extends VelocityDriveState {
    private final Vector3 position;
    private final Vector2 target;
    private final double power;
    private final Terminator terminator;

    private final PIDControl lineControl, rotationControl;
    private final PIDControl2 nearTargetControl;

    public LineDrive(StateMachine stateMachine, RobotDrive drive, Vector2 target, Vector3 position, double power, Terminator terminator) {
        super(stateMachine, drive);
        this.power = power;
        this.position = position;
        this.target = target;
        this.terminator = terminator;

        lineControl = new PIDControl(1, 0, 0);
        rotationControl = new PIDControl(1, 0, 0);
        nearTargetControl = new PIDControl2(1, 0, 0);
    }

    @Override
    public void update(ReadData data) {

        if(new Vector2(position).distanceTo(target)<1){

        }
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return null;
    }
}
