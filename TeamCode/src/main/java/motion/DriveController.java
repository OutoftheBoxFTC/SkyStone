package motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.MathUtil;
import math.Vector2;
import math.Vector3;
import motion.terminator.Terminator;
import state.StateMachine;

public class DriveController extends VelocityDriveState {
    protected final Vector3 target, position;
    protected final double aoa, power;
    protected final Terminator terminator;
    protected final boolean strictOrientation;

    protected final PIDControl lineControl, rotationControl;
    protected final PIDControl2 nearTargetControl;

    protected double lineAngle;

    public DriveController(StateMachine stateMachine, RobotDrive drive, Vector3 target, Vector3 position, double aoa, double power, Terminator terminator, boolean strictOrientation) {
        super(stateMachine, drive);
        this.aoa = aoa;
        this.power = power;
        this.position = position;
        this.target = target;
        this.terminator = terminator;
        this.strictOrientation = strictOrientation;

        lineControl = new PIDControl(1, 0, 0);
        rotationControl = new PIDControl(1, 0, 0);
        nearTargetControl = new PIDControl2(1, 0, 0);
    }

    @Override
    public void init(ReadData data) {
        Vector2 position = new Vector2(this.position),
                target = new Vector2(this.target);
        lineAngle = position.angleTo(target);
    }

    @Override
    public void update(ReadData data) {
        double angle = position.getC()+aoa;
        double rotation = rotationControl.evaluation(MathUtil.angleDelta(angle, lineAngle));

    }

    @Override
    protected Vector3 getRobotVelocity() {
        return null;
    }
}
