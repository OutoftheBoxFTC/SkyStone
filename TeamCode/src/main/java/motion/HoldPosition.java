package motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.Vector2;
import math.Vector3;
import state.StateMachine;
import state.motion.VelocityDriveState;

public class HoldPosition extends VelocityDriveState {
    private PIDControl rotationControl;
    private PIDControl2 translationControl;
    private Vector3 velocity, position;

    public HoldPosition(StateMachine stateMachine, RobotDrive robotDrive, Vector3 position) {
        super(stateMachine, robotDrive);
        rotationControl = new PIDControl(1, 0, 0);
        translationControl = new PIDControl2(1, 0, 0);
        velocity = Vector3.ZERO();
        this.position = position;
    }

    @Override
    public void init(ReadData data) {
        super.init(data);
        translationControl.reset();
        rotationControl.reset();
    }

    @Override
    public void update(ReadData data) {
        super.update(data);
        Vector2 translationalVelocity = translationControl.evaluation(new Vector2(position), data.getHub1BulkTime());
        double rotationVelocity = rotationControl.evaluation(position.getC(), data.getHub1BulkTime());
        velocity = new Vector3(translationalVelocity.getA(), translationalVelocity.getB(), rotationVelocity);
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocity;
    }
}
