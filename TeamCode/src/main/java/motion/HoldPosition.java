package motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.MathUtil;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import state.StateMachine;
import state.motion.VelocityDriveState;

public class HoldPosition extends VelocityDriveState {
    private PIDControl rotationControl;
    private PIDControl2 translationControl;
    private Vector3 velocity, position, target;

    public HoldPosition(StateMachine stateMachine, RobotDrive robotDrive, Vector3 interpolatedPosition, Vector3 target) {
        super(stateMachine, robotDrive);
        rotationControl = new PIDControl(0.3, 0.1, 0);
        translationControl = new PIDControl2(0.025, 0.1, 0);
        velocity = Vector3.ZERO();
        this.target = target;
        this.position = interpolatedPosition;
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
        Vector3 error = new Vector3(new Vector2(target).add(new Vector2(position).scale(-1)), MathUtil.angleDelta(position.getC(), target.getC()));
        Vector2 translationalVelocity = translationControl.evaluation(new Vector2(error), data.getHub1BulkTime());
        double rotationVelocity = rotationControl.evaluation(error.getC(), data.getHub1BulkTime());
        velocity = transformToRobot(new Vector3(translationalVelocity, rotationVelocity));
    }

    private Vector3 transformToRobot(Vector3 fieldVelocity){
        Matrix22 rotationInverse = MathUtil.rotationMatrix(position.getC());
        Vector2 robotTranslationVelocity =  rotationInverse.transform(new Vector2(fieldVelocity));
        return new Vector3(robotTranslationVelocity, fieldVelocity.getC());
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocity;
    }
}
