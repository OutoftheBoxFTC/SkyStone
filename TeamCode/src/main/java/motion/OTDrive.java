package motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.Vector3;
import state.StateMachine;
import state.motion.VelocityDriveState;

public class OTDrive extends VelocityDriveState {
    private static final double RADIUS = 9.25, V_MAX = 39.1433457, TIME_ADV = 0.001;

    private Vector3 target, position, previousPosition, velocity;
    private long lastTime;
    private double vMax;

    public OTDrive(StateMachine stateMachine, RobotDrive robotDrive, Vector3 target, double power, Vector3 position) {
        super(stateMachine, robotDrive);
        this.position = position;
        previousPosition = Vector3.ZERO();
        this.vMax = power*V_MAX;
        this.target = target;
    }

    @Override
    public void init(ReadData data) {
        super.init(data);
        lastTime = data.getHub1BulkTime();
    }

    @Override
    public void update(ReadData data) {
        Vector3 newPosition = position.clone();
        if(data.getHub1BulkTime()!=lastTime) {
            double dt = (data.getHub1BulkTime() - lastTime) / 1.0e9;
            Vector3 velocity = position.subtract(previousPosition).scale(1 / dt);
            newPosition = newPosition.add(velocity.scale(TIME_ADV));
        }
        this.velocity.set(getVelocity(newPosition));

        previousPosition.set(position);
        lastTime = data.getHub1BulkTime();
    }

    private Vector3 getVelocity(Vector3 position){

        //TODO finish me
        return null;
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocity;
    }
}
