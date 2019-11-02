package opmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Odometer.SimpleOdometer;
import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import math.Vector4;
import motion.VelocityDriveState;
import state.DriveState;
import state.LogicState;
@TeleOp
public class OdometerTeleOp extends BasicOpmode {
    Vector3 position, velocity;
    MecanumDrive drive;
    public OdometerTeleOp() {
        super(1, false);
    }

    @Override
    protected void setup() {
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.enableDevice(Hardware.HardwareDevice.GYRO);
        robot.enableDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.enableDevice(Hardware.HardwareDevice.HUB_2_BULK);
        robot.enableDevice(Hardware.HardwareDevice.OTHER_MOTORS);
        final SimpleOdometer odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity, telemetry);
        drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        HashMap<String, LogicState> logicStates = new HashMap<>();
        logicStates.put("Orientation", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                odometer.update(data);
                telemetry.setHeader("Position", position);
            }
        });
        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("Drive", new VelocityDriveState(stateMachine, drive) {
            @Override
            protected Vector3 getRobotVelocity() {
                return new Vector3(gamepad1.leftStickX, gamepad1.leftStickY, -gamepad1.rightStickX);
            }
        });
        driveStates.put("FieldCentric", new VelocityDriveState(stateMachine, drive) {
            Vector3 velocity = Vector3.ZERO();
            @Override
            public void update(ReadData data){
                double r = Math.sqrt(gamepad1.leftStickX * gamepad1.leftStickX + gamepad1.leftStickY * gamepad1.leftStickY);
                double theta = Math.atan2(gamepad1.leftStickY, gamepad1.leftStickX) + data.getGyro();
                velocity.set(new Vector3(r * Math.cos(theta), r * Math.sin(theta), -gamepad1.rightStickX));
            }
            @Override
            public Vector3 getRobotVelocity() {
                return velocity;
            }
        });
        logicStates.put("intake", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(gamepad1.rightTrigger != 0) {
                    robot.intake(gamepad1.rightTrigger);
                }else {
                    robot.variedIntake(gamepad1.leftTrigger);
                }
            }
        });
        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(opModeIsActive()){
                    stateMachine.activateLogic("Orientation");
                    stateMachine.setActiveDriveState("FieldCentric");
                    stateMachine.activateLogic("intake");
                    odometer.start(data);
                    robot.turnBrakeOff();
                    deactivateThis();

                }
            }
        });
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);
        stateMachine.activateLogic("init");
    }
}
