package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import motion.FieldCentricDriverControl;
import odometry.Odometer;
import odometry.SimpleOdometer;
import state.DriveState;
import state.LogicState;
import state.Orientation;

@TeleOp
public class OdometerLoggingTests extends BasicOpmode {
    Odometer odometer;
    Vector3 position, velocity;
    public OdometerLoggingTests() {
        super(0, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.GYRO);
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.registerDevice(Hardware.HardwareDevice.HUB_1_BULK);
        MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        FieldCentricDriverControl fieldCentricDriverControl = new FieldCentricDriverControl(position, gamepad1, stateMachine, drive);
        telemetry.enableLogger();
        odometer = new SimpleOdometer();
        HashMap<String, LogicState> logicStates = new HashMap<>();
        HashMap<String, DriveState> driveStates = new HashMap<>();
        position = new Vector3(0, 0, 0);
        velocity = new Vector3(0, 0, 0);
        logicStates.put("Orientation", new Orientation(stateMachine, odometer, position, velocity){
            @Override
            public void update(ReadData data) {
                super.update(data);
                telemetry.setHeader("position", position);
            }
        });
        logicStates.put("Init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    stateMachine.activateLogic("Orientation");
                    stateMachine.setActiveDriveState("FieldCentric");
                    deactivateThis();
                }
            }
        });
        driveStates.put("FieldCentric", fieldCentricDriverControl);
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);
        stateMachine.activateLogic("Init");
    }
}
