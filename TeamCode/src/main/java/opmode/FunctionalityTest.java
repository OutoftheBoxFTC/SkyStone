package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import hardware.Hardware;
import hardware.ReadData;
import math.Vector4;
import state.DriveState;
import state.LogicState;

/**
 * This class is a raw debug class of all sensors/functionality to test against expected behaviour.
 * Y: Front left wheel fwd (a)
 * X: Front right wheel fwd (b)
 * A: Back left wheel fwd (c)
 * B: Back right wheel fwd (d)
 * Right Bumper: Right Lift
 * Left Bumper: Left Lift
 * Right Trigger: Upper Servo
 * Left Trigger: Lower Servo
 *
 *
 * When moving robot fwd, left and right increases
 * When rotating cc, aux and right increases, left decreases
 *
 * Gyro starts at 0, increases to 2pi, and returns to 0 after 1 full cc rotation
 */
@TeleOp(name = "Functionality Test")
public class FunctionalityTest extends BasicOpmode {
    public FunctionalityTest() {
        super(1, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.registerDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.registerDevice(Hardware.HardwareDevice.HUB_2_BULK);
        robot.registerDevice(Hardware.HardwareDevice.GYRO);
        //robot.registerDevice(Hardware.HardwareDevice.WEBCAM_1);
        robot.registerDevice(Hardware.HardwareDevice.INTAKE_MOTORS);
        robot.registerDevice(Hardware.HardwareDevice.INTAKE_SERVOS);
        //robot.registerDevice(Hardware.HardwareDevice.LIFT_MOTORS);
        //robot.registerDevices(Hardware.HardwareDevice.LIFT_SERVOS);

        HashMap<String, LogicState> logicStates = new HashMap<>();
        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.activateLogic("Sensor Readout");
                    stateMachine.setActiveDriveState("Controller Drive");
                    telemetry.clearAllHeadersExcept("Main Loop FPS", "Hardware FPS", "Activated Logic States", "vel");
                }
            }
        });
        logicStates.put("Sensor Readout", new LogicState(stateMachine) {
            boolean rightFlipped = false;
            double previousRight = 0;
            @Override
            public void update(ReadData data) {
                if(previousRight - data.getRight() > 1000){
                    rightFlipped = true;
                }
                telemetry.setHeader("left", String.valueOf(data.getLeft()));
                telemetry.setHeader("Right", String.valueOf(data.getRight()));
                telemetry.setHeader("Aux", String.valueOf(data.getAux()));
                telemetry.setHeader("Gyro", String.valueOf(data.getGyro()));
                telemetry.setHeader("Flipped", rightFlipped);
                if(isStopRequested()){
                    stateMachine.deactivateLogic("Sensor Readout");
                }
            }
        });
        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("Controller Drive", new DriveState(stateMachine) {
            @Override
            public Vector4 getWheelVelocities() {
                //a, b, c, d
                return new Vector4(gamepad1.y.isPressed()?0.5:0, gamepad1.x.isPressed()?0.5:0, gamepad1.a.isPressed()?0.5:0, gamepad1.b.isPressed()?0.5:0);
            }
        });
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);
        stateMachine.activateLogic("init");
    }
}