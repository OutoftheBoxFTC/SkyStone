package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import math.Vector4;
import state.motion.FieldCentricDriverControl;
import state.DriveState;
import state.LogicState;

@TeleOp(name = "Teleop")
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
                    stateMachine.activateLogic("intakeOpen");
                    stateMachine.setActiveDriveState("drive");
                }
            }
        });

        logicStates.put("intakeOpen", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(gamepad1.rightBumper.isActive()&&gamepad1.rightBumper.isUpdated()){
                    deactivateThis();
                    stateMachine.activateLogic("intakeClosed");
                    robot.intakeServos(0.37, 0.37);
                }
            }
        });
        logicStates.put("intakeClosed", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(gamepad1.rightBumper.isActive()&&gamepad1.rightBumper.isUpdated()){
                    deactivateThis();
                    stateMachine.activateLogic("intakeOpen");
                    robot.intakeServos(0, 0);
                }
            }
        });

        logicStates.put("run", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                position.setC(data.getGyro());
                if(gamepad1.rightTrigger>0){
                    robot.intake(-gamepad1.rightTrigger, -gamepad1.rightTrigger);
                }
                else {
                    robot.intake(gamepad1.leftTrigger, gamepad1.leftTrigger);
                }
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
