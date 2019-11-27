package motion;

import drivetrain.RobotDrive;
import hardware.ReadData;
import math.MathUtil;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import state.StateMachine;
import state.motion.VelocityDriveState;

public class CircleDrive extends VelocityDriveState {
    private static final double RADIUS = 9.25, V_MAX = 39.1433457, TIME_ADV = 0.001;

    private Vector3 position, velocity, previousPosition, start;
    private Vector2 targetPoint;
    private double power;
    private long lastTime;

    private Vector2 center;
    private double signedRadius;

    private PIDControl radialControl, rotationControl;

    private double ffRotation;

    public CircleDrive(StateMachine stateMachine, RobotDrive robotDrive, Vector3 position, Vector3 start, Vector2 targetPoint, double power) {
        super(stateMachine, robotDrive);
        this.position = position;
        this.targetPoint = targetPoint;
        this.power = power;
        this.start = start;
        center = Vector2.ZERO();
        radialControl = new PIDControl(0.1, 0, 0);
        rotationControl = new PIDControl(0.3, 0.2, 0, false);
    }

    public CircleDrive(StateMachine stateMachine, RobotDrive drive, Vector3 position, Vector2 targetPoint, double power){
        this(stateMachine, drive, position, null, targetPoint, power);
    }

    @Override
    public void init(ReadData data) {
        super.init(data);
        if(start == null){
            start = new Vector3(position);
        }
        lastTime = data.getHub1BulkTime();
        Vector2 delta = MathUtil.rotationMatrix(-start.getC()).transform(targetPoint.add(new Vector2(start).scale(-1)));
        this.signedRadius =(delta.getA()*delta.getA()+delta.getB()*delta.getB())/2/delta.getB();
        Vector2 center = new Vector2(0, signedRadius);
        this.center.set(MathUtil.rotationMatrix(start.getC()).transform(center).add(new Vector2(start)));
        ffRotation = RADIUS/signedRadius*power;
        previousPosition = new Vector3(position);
    }

    @Override
    public void update(ReadData data) {
        super.update(data);
        Vector3 newPosition = position.clone();
        if(data.getHub1BulkTime()!=lastTime) {
            double dt = (data.getHub1BulkTime() - lastTime) / 1.0e9;
            Vector3 velocity = position.subtract(previousPosition).scale(1 / dt);
            newPosition = newPosition.add(velocity.scale(TIME_ADV));
        }
        velocity = getVelocity(newPosition, data.getHub1BulkTime());
        previousPosition.set(position);
    }

    private Vector3 getVelocity(Vector3 position, long timeStamp){
        double angleToPosition = center.angleTo(new Vector2(position));
        double targetRotation = (angleToPosition+Math.PI/2*MathUtil.sgn(signedRadius));
        Vector2 feedFwdTranslation = new Vector2(power, 0);
        feedFwdTranslation = MathUtil.rotationMatrix(targetRotation).transform(feedFwdTranslation);
        Vector3 feedFwd = new Vector3(feedFwdTranslation, ffRotation);

        double radialError = Math.abs(signedRadius)-center.distanceTo(new Vector2(position));
        double radialCorrection = radialControl.evaluation(radialError, timeStamp);
        double rotationError = MathUtil.angleDelta(position.getC(), targetRotation);
        double rotationCorrection = rotationControl.evaluation(rotationError, timeStamp);
        Vector2 radialCorrectionVector = new Vector2(radialCorrection, 0);
        radialCorrectionVector = MathUtil.rotationMatrix(angleToPosition).transform(radialCorrectionVector);
        Vector3 correction = new Vector3(radialCorrectionVector, rotationCorrection);

        return transformToRobot(feedFwd.add(correction));
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
