package opmode;

import android.bluetooth.BluetoothDevice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Odometer.SimpleOdometer;
import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import math.Vector4;
import motion.DriveToPointBuilder;
import motion.Terminator.OrientationTerminator;
import motion.VelocityDriveState;
import state.DriveState;
import state.LogicState;
import state.motion.TurnCorrectionVector;

@TeleOp(name="BlueAutonomous")
public class BlueAutonomous extends BasicOpmode {
    public BlueAutonomous(){
        super(1, false);
    }
    @Override
    protected void setup() {
        final SimpleOdometer odometer;
        final Vector3 position, velocity;
        final MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.enableDevice(Hardware.HardwareDevice.HUB_2_BULK);
        robot.enableDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.enableDevice(Hardware.HardwareDevice.GYRO);
        robot.enableDevice(Hardware.HardwareDevice.SERVOS);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity, telemetry);
        DriveToPointBuilder builder = new DriveToPointBuilder(stateMachine, new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1), position, velocity, odometer);
        HashMap<String, DriveState> driveStates = new HashMap<>();
        HashMap<String, LogicState> states = new HashMap<>();
        states.put("Orientation", new LogicState(stateMachine){
            @Override
            public void update(ReadData data) {
                odometer.update(data);
                telemetry.setHeader("x", position.getA());
                telemetry.setHeader("y", position.getB());
                telemetry.setHeader("r", position.getC());
            }
        });
        states.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(opModeIsActive()){
                    stateMachine.activateLogic("Tracking");
                    stateMachine.setActiveDriveState("driveToFoundation");
                    stateMachine.activateLogic("Orientation");
                    stateMachine.activateLogic("latchOn");
                    odometer.start(data);
                    deactivateThis();
                }
            }
        });
        states.put("Tracking", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("Position", position);
                telemetry.setHeader("Velocity", velocity);
                telemetry.setHeader("raw", new Vector3(data.getLeft(), data.getRight(), data.getAux()));
            }
        });
        driveStates.put("end", new VelocityDriveState(stateMachine, new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1)) {

            @Override
            protected Vector3 getRobotVelocity() {
                return Vector3.ZERO();
            }
        });

        final OrientationTerminator driveToFoundationTerminator = new OrientationTerminator(position, new Vector3(0, -18, 180), 4, 1);
        driveStates.put("driveToFoundation", builder.create(new Vector3(0, -18, 0), 0, driveToFoundationTerminator, 0.25, "waitForServos"));
        states.put("latchOn", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(driveToFoundationTerminator.shouldTerminate(data)){
                   robot.latchOn();
                   deactivateThis();
                }
            }
        });
        driveStates.put("waitForServos", new VelocityDriveState(stateMachine, drive) {
            long timer;
            @Override
            public void init(ReadData data){
                timer = System.currentTimeMillis() + 1500;
            }
            @Override
            public void update(ReadData data){
                if(System.currentTimeMillis() > timer){
                    deactivateThis();
                    stateMachine.setActiveDriveState("driveBack");
                }
            }
            @Override
            protected Vector3 getRobotVelocity() {
                return Vector3.ZERO();
            }
        });
        final OrientationTerminator driveFoundationBack = new OrientationTerminator(position, new Vector3(0, 18, -90), 4, 1);
        driveStates.put("driveBack", builder.create(new Vector3(0, 18, 0), 0, driveFoundationBack, 0.25, "end"));
        driveStates.put("turnToSkystones", builder.turn(0.01, -90, "end", driveFoundationBack));
        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("init");
    }
}
