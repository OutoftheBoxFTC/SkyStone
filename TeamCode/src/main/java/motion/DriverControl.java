package motion;

import drivetrain.MecanumDrive;
import hardware.controller.SmartGamepad;
import math.Vector3;
import state.StateMachine;
import state.motion.VelocityDriveState;

public class DriverControl extends VelocityDriveState {
    private SmartGamepad driverController;

    public DriverControl(SmartGamepad driverController, StateMachine stateMachine, MecanumDrive drive) {
        super(stateMachine, drive);
        this.driverController = driverController;
    }

    @Override
    public Vector3 getRobotVelocity() {
        return new Vector3(-driverController.rightStickY, -driverController.rightStickX, -driverController.leftStickX);
    }
}