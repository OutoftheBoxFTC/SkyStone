package opmode;

import android.bluetooth.BluetoothDevice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Odometer.SimpleOdometer;
import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import hardware.Sensors.Pixycam2Provider;
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
        robot.enableDevice(Hardware.HardwareDevice.PIXYCAM);
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
                    stateMachine.activateLogic("latchOff");
                    stateMachine.activateLogic("pixyData");
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

        final OrientationTerminator driveToFoundationTerminator = new OrientationTerminator(position, new Vector3(9, -17, 180), 4, 1);
        driveStates.put("driveToFoundation", builder.create(new Vector3(9, -17, 0), 0, driveToFoundationTerminator, 0.25, "end"));
        states.put("latchOn", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(driveToFoundationTerminator.shouldTerminate(data) && timer == 0){
                   robot.latchOn();
                   timer = System.currentTimeMillis() + 1500;
                }
                if(timer != 0 && System.currentTimeMillis() > timer){
                    stateMachine.setActiveDriveState("driveBack");
                    deactivateThis();
                }
                telemetry.setHeader("Time", timer - System.currentTimeMillis());
            }
        });
        final OrientationTerminator driveFoundationBack = new OrientationTerminator(position, new Vector3(9, -1, 90), 4, 25);
        driveStates.put("driveBack", builder.create(new Vector3(9, -1, 0), 0, driveFoundationBack, 0.7, "turnToSkystones"));
        states.put("latchOff", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(driveFoundationBack.shouldTerminateRotation() && timer == 0){
                    robot.latchOff();
                    timer = System.currentTimeMillis() + 1500;
                }
                if(timer != 0 && System.currentTimeMillis() > timer){
                    stateMachine.setActiveDriveState("driveToSkystones");
                    deactivateThis();
                }
                telemetry.setHeader("Time", timer - System.currentTimeMillis());
            }
        });
        final OrientationTerminator driveToSkystones = new OrientationTerminator(position, new Vector3(-10, -8, -90), 4, 1);
        driveStates.put("turnToSkystones", builder.turn(0.01, 90, "driveToSkystones", driveToSkystones));
        driveStates.put("driveToSkystones", builder.create(new Vector3(-5, -8, 90), 0, driveToSkystones, 1, "turnToSeeSkystones"));
        final OrientationTerminator turnToSeeSkystones = new OrientationTerminator(position, new Vector3(0, 0, 10), 0, 1);
        driveStates.put("turnToSeeSkystones", builder.turn(0.01, 10, "end", turnToSeeSkystones));
        states.put("pixyData", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(driveToSkystones.shouldTerminate(data) && timer == 0) {
                    timer = 1;
                }
                if(timer == 1){
                    telemetry.setHeader("Y", robot.getPixy().getY());
                }
            }
        });
        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("init");
    }
}
