package state.motion;

import drivetrain.MecanumDrive;
import hardware.controller.SmartGamepad;
import math.MathUtil;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import motion.DriverControl;
import state.StateMachine;

public class FieldCentricDriverControl extends DriverControl {
    private Vector3 position;

    public FieldCentricDriverControl(Vector3 actingPosition, SmartGamepad driverController, StateMachine stateMachine, MecanumDrive drive){
        super(driverController, stateMachine, drive);
        this.position = actingPosition;
    }

    @Override
    public Vector3 getRobotVelocity() {
        return transformToRobot(super.getRobotVelocity());
    }

    private Vector3 transformToRobot(Vector3 fieldVelocity){
        Matrix22 rotationInverse = MathUtil.rotationMatrix(position.getC());
        Vector2 robotTranslationVelocity =  rotationInverse.transform(new Vector2(fieldVelocity));
        return new Vector3(robotTranslationVelocity, fieldVelocity.getC());
    }
}
