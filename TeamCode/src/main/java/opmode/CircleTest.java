package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector2;
import math.Vector3;
import motion.CircleDrive;
import motion.terminator.PositionTerminator;
import motion.terminator.SensitivityTerminator;
import state.DriveState;
import state.LogicState;

@TeleOp(name = "Circle Test")
public class CircleTest extends BasicOpmode {
    public CircleTest() {
        super(0, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS)
                .registerDevices(Hardware.HardwareDevice.HUB_1_BULK)
                .registerDevices(Hardware.HardwareDevice.HUB_2_BULK);

        final MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        final Vector3 position = new Vector3(0, 0, Math.PI/2);

        HashMap<String, LogicState> logicStates = new HashMap<>();
        HashMap<String, DriveState> driveStates = new HashMap<>();

        final SensitivityTerminator circleTerminator = new PositionTerminator(position, new Vector3(-40, 0, 0), 0.1, 0.1, 0.2).getDistanceTerminator();

        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.setActiveDriveState("circle");
                }
            }
        });

        logicStates.put("run", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("should terminate", circleTerminator.shouldTerminate(new Vector2(position).distanceTo(new Vector2(-40, 0)), data.getHub1BulkTime()) + "");
            }
        });


        driveStates.put("circle", new CircleDrive(stateMachine, drive, position, new Vector2(-40, 0), 1, telemetry));


        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(logicStates);
        stateMachine.activateLogic("init");
    }
}
