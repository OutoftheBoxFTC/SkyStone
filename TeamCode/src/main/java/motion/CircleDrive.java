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

    private Vector3 position, velocity, previousPosition;
    private Vector2 targetPoint;
    private double power;
    private long lastTime;

    private Vector2 center;
    private double signedRadius;

    private PIDControl radialControl, rotationControl;

    private double ffRotation;

    public CircleDrive(StateMachine stateMachine, RobotDrive robotDrive, Vector3 position, Vector2 targetPoint, double power) {
        super(stateMachine, robotDrive);
        this.position = position;
        this.targetPoint = targetPoint;
        this.power = power;
        center = Vector2.ZERO();
        radialControl = new PIDControl(0.1, 0, 0);
        rotationControl = new PIDControl(0.3, 0.2, 0, false);
    }

    @Override
    public void init(ReadData data) {
        super.init(data);
        lastTime = data.getHub1BulkTime();
        Vector2 delta = rotationMatrix(-position.getC()).transform(targetPoint.add(new Vector2(position).scale(-1)));
        this.signedRadius =(delta.getA()*delta.getA()+delta.getB()*delta.getB())/2/delta.getB();
        if(Math.abs(delta.getB())<0.01){
            signedRadius = Double.POSITIVE_INFINITY;
            ffRotation = 0;
            center = new Vector2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        } else {
            //positive when angle change to target is positive. Negative respectively.
            Vector2 center = new Vector2(0, signedRadius);
            this.center.set(rotationMatrix(position.getC()).transform(center).add(new Vector2(position)));
            ffRotation = RADIUS / signedRadius * power;
        }
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
        previousPosition = position;
    }

    private Vector3 getVelocity(Vector3 position, long timeStamp){
        double angleToPosition = center.angleTo(new Vector2(position));
        double targetRotation = (angleToPosition+Math.PI/2*MathUtil.sgn(signedRadius));
        if(signedRadius == Double.POSITIVE_INFINITY){
            angleToPosition = 0;
        }
        Vector2 feedFwdTranslation = new Vector2(power, 0);
        feedFwdTranslation = rotationMatrix(targetRotation).transform(feedFwdTranslation);
        Vector3 feedFwd = new Vector3(feedFwdTranslation, ffRotation);

        double distance = center.distanceTo(new Vector2(position));
        double radialCorrection = radialControl.evaluation(Math.abs(signedRadius)-distance, timeStamp);

        double rotationError = MathUtil.angleDelta(position.getC(), targetRotation);
        double rotationCorrection = rotationControl.evaluation(rotationError, timeStamp);
        Vector2 radialCorrectionVector = new Vector2(radialCorrection, 0);
        radialCorrectionVector = rotationMatrix(angleToPosition).transform(radialCorrectionVector);//rotate so +x is in +radial direction
        Vector3 correction = new Vector3(radialCorrectionVector, rotationCorrection);
        return transformToRobot(feedFwd.add(correction));
    }

    private Vector3 transformToRobot(Vector3 fieldVelocity){
        double sine = Math.sin(position.getC()), cos = Math.cos(position.getC());
        Matrix22 rotationInverse = new Matrix22(cos, sine, -sine, cos);
        Vector2 robotTranslationVelocity =  rotationInverse.transform(new Vector2(fieldVelocity));
        return new Vector3(robotTranslationVelocity, fieldVelocity.getC());
    }

    @Override
    protected Vector3 getRobotVelocity() {
        return velocity;
    }

    private Matrix22 rotationMatrix(double rotation){
        double cos = Math.cos(rotation),
                sine = Math.sin(rotation);
        return new Matrix22(cos, -sine, sine, cos);
    }
}
