package motion;

import drivetrain.MecanumDrive;
import hardware.controller.SmartGamepad;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import state.StateMachine;

public class FieldCentricDriverControl extends DriverControl {
    private Vector3 position;

    public FieldCentricDriverControl(Vector3 position, SmartGamepad driverController, StateMachine stateMachine, MecanumDrive drive){
        super(driverController, stateMachine, drive);
        this.position = position;
    }

    @Override
    public Vector3 getRobotVelocity() {
        return transformToRobot(super.getRobotVelocity());
    }

    private Vector3 transformToRobot(Vector3 fieldVelocity){
        double sine = Math.sin(position.getC()), cos = Math.cos(position.getC());
        Matrix22 rotationInverse = new Matrix22(cos, sine, -sine, cos);
        Vector2 robotTranslationVelocity =  rotationInverse.transform(new Vector2(fieldVelocity));
        return new Vector3(robotTranslationVelocity, fieldVelocity.getC());
    }
}
