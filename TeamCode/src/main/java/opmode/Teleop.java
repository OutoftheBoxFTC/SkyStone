package opmode;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import math.Vector4;
import motion.FieldCentricDriverControl;
import state.DriveState;
import state.LogicState;

public class Teleop extends BasicOpmode {

    public Teleop() {
        super(1, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS)
                .registerDevices(Hardware.HardwareDevice.INTAKE_MOTORS)
                .registerDevices(Hardware.HardwareDevice.INTAKE_SERVOS)
                .registerDevice(Hardware.HardwareDevice.GYRO);
        final MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        final Vector3 position = new Vector3(0, 0, 0);

        HashMap<String, LogicState> logicStates = new HashMap<>();
        HashMap<String, DriveState> driveStates = new HashMap<>();

        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.activateLogic("run");
                    stateMachine.setActiveDriveState("drive");
                }
            }
        });

        logicStates.put("run", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                position.setC(data.getGyro());

            }
        });


        driveStates.put("drive", new FieldCentricDriverControl(position, gamepad1, stateMachine, drive));
        driveStates.put("none", new DriveState(stateMachine) {
            @Override
            public Vector4 getWheelVelocities() {
                return Vector4.ZERO();
            }
        });


        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);
        stateMachine.activateLogic("init");
        stateMachine.setActiveDriveState("none");
    }
}
