package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import motion.DriverControl;
import odometry.SimpleOdometer;
import state.DriveState;
import state.LogicState;
import state.Orientation;
import state.motion.FieldCentricDriverControl;
import state.motion.VelocityDriveState;

@TeleOp(name = "Driver Controller Test")
public class DriverControllerTest extends BasicOpmode {
    private MecanumDrive drive;
    private Vector3 position;

    public DriverControllerTest() {
        super(0.3, false);
        position = new Vector3(0, 0, 0);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.registerDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.registerDevice(Hardware.HardwareDevice.HUB_2_BULK);
        //telemetry.enableLogger();
        HashMap<String, LogicState> logicStates = new HashMap<>();
        HashMap<String, DriveState> driveStates = new HashMap<>();
        drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if (isStarted()){
                    stateMachine.setActiveDriveState("robot centric");
                    stateMachine.activateLogic("run");
                    stateMachine.activateLogic("orientation");
                    deactivateThis();
                }
            }
        });
        logicStates.put("orientation", new Orientation(stateMachine, new SimpleOdometer(), position){
            @Override
            public void update(ReadData data) {
                super.update(data);
                telemetry.setHeader("X", position.getA());
                telemetry.setHeader("Y", position.getB());
                telemetry.setHeader("R", Math.toDegrees(position.getC()));
            }
        });
        logicStates.put("run", new LogicState(stateMachine) {
            double previousRight = 0;
            boolean rightFlipped = false;
            @Override
            public void init(ReadData data) {

            }

            @Override
            public void update(ReadData data) {
                if(gamepad1.a.isPressed()&&gamepad1.a.isUpdated()){
                    stateMachine.setActiveDriveState("field centric");
                } else if(gamepad1.b.isPressed()&&gamepad1.b.isUpdated()){
                    stateMachine.setActiveDriveState("robot centric");
                }

                if(previousRight - data.getRight() > 1000){
                    rightFlipped = true;
                }
                telemetry.setHeader("Right", data.getRight());
                telemetry.setHeader("Left", data.getLeft());
                telemetry.setHeader("Flipped", String.valueOf(rightFlipped));
                previousRight = data.getRight();
            }
        });

        driveStates.put("none", new VelocityDriveState(stateMachine, drive) {
            @Override
            public Vector3 getRobotVelocity() {
                return new Vector3(0, 0, 0);
            }
        });
        driveStates.put("robot centric", new DriverControl(gamepad1, stateMachine, drive));
        driveStates.put("field centric", new FieldCentricDriverControl(position, gamepad1, stateMachine, drive));

        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);

        stateMachine.setActiveDriveState("none");
        stateMachine.activateLogic("init");
    }
}