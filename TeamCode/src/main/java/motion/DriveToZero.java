package motion;

import drivetrain.MecanumDrive;
import hardware.ReadData;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import state.StateMachine;

public class DriveToZero extends VelocityDriveState {
    //TODO debug and check this. Getting a seamless transition (where it doesn't return 0 on the first iteration) is key
    private static final double DISTANCE_THRESHOLD = 5;//translation pid kicks in within 5" of the target

    private Vector3 position;
    private PIDControl2 translationControl;
    private PIDControl rotationControl;
    private Vector3 robotVelocity;

    private boolean wasInDistance;

    public DriveToZero(Vector3 position, PIDControl2 translationControl, PIDControl rotationControl, StateMachine stateMachine, MecanumDrive drive){
        super(stateMachine, drive);
        this.position = position;
        this.translationControl = translationControl;
        this.rotationControl = rotationControl;
    }

    @Override
    public void init(ReadData data) {
        wasInDistance = false;
    }

    @Override
    public Vector3 getRobotVelocity() {
        return robotVelocity;

    }

    private Vector3 transformToRobot(Vector3 fieldVelocity){
        double sine = Math.sin(position.getC()), cos = Math.sqrt(1-sine*sine);
        Matrix22 rotationInverse = new Matrix22(cos, sine, -sine, cos);
        Vector2 robotTranslationVelocity =  rotationInverse.transform(new Vector2(fieldVelocity));
        return new Vector3(robotTranslationVelocity, fieldVelocity.getC());
    }

    @Override
    public void update(ReadData data) {
        Vector2 fieldTranslation = new Vector2(position);
        double distance = fieldTranslation.length(),
                angleToZero = fieldTranslation.angleTo(Vector2.ZERO());
        if(distance <= DISTANCE_THRESHOLD){
            if(!wasInDistance){
                translationControl.reset();
                rotationControl.reset();
                wasInDistance = true;
            }
            Vector3 fieldVelocity = new Vector3(translationControl.evaluation(fieldTranslation), rotationControl.evaluation(position.getC()));
            robotVelocity = transformToRobot(fieldVelocity);
        } else {
            if(wasInDistance) {
                rotationControl.reset();
                wasInDistance = false;
            }
            Vector2 translationalVelocity = new Matrix22(fieldTranslation.normalize()).transform(new Vector2(DISTANCE_THRESHOLD, DISTANCE_THRESHOLD)).scale(translationControl.getKp());
            Vector3 fieldVelocity = new Vector3(translationalVelocity, rotationControl.evaluation(angleToZero));
            robotVelocity = transformToRobot(fieldVelocity);
        }
    }
}