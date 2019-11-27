package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import motion.HoldPosition;
import odometry.SimpleOdometer;
import state.DriveState;
import state.LogicState;
import state.Orientation;

@TeleOp(name = "Hold Position Test")
public class HoldPositionTest extends BasicOpmode {

    public HoldPositionTest() {
        super(0.5, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.registerDevice(Hardware.HardwareDevice.HUB_2_BULK);
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);

        final Vector3 position = Vector3.ZERO();
        final MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);

        HashMap<String, LogicState> logicStates = new HashMap<>();
        HashMap<String, DriveState> driveStates = new HashMap<>();

        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.setActiveDriveState("holdPosition");
                }
            }
        });
        logicStates.put("orientation", new Orientation(stateMachine, new SimpleOdometer(), position){
            @Override
            public void update(ReadData data) {
                super.update(data);
                telemetry.setHeader("position", position);
            }
        });
        driveStates.put("holdPosition", new HoldPosition(stateMachine, drive, position, new Vector3(0, 0, Math.PI/2)));
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);

        stateMachine.activateLogic("orientation", 0);
        stateMachine.activateLogic("init");
    }
}
