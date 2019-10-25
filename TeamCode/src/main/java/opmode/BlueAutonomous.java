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
import motion.Terminator.CombinedTerminator;
import motion.Terminator.OrientationTerminator;
import motion.Terminator.PixyTerminator;
import motion.VelocityDriveState;
import state.DriveState;
import state.LogicState;
import state.motion.TurnCorrectionVector;

@TeleOp(name="BlueAutonomous")
public class BlueAutonomous extends BasicOpmode {
    public BlueAutonomous(){
        super(1, false);
    }
    Vector3 skystoneCoords =  Vector3.ZERO();
    @Override
    protected void setup() {
        final SimpleOdometer odometer;
        final Vector3 position, velocity;
        final MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.enableDevice(Hardware.HardwareDevice.HUB_2_BULK);
        robot.enableDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.enableDevice(Hardware.HardwareDevice.OTHER_MOTORS);
        robot.enableDevice(Hardware.HardwareDevice.GYRO);
        robot.enableDevice(Hardware.HardwareDevice.SERVOS);
        robot.enableDevice(Hardware.HardwareDevice.PIXYCAM);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity, telemetry);
        final DriveToPointBuilder builder = new DriveToPointBuilder(stateMachine, new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1), position, velocity, odometer);
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
                    robot.disableDevice(Hardware.HardwareDevice.PIXYCAM);
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
        driveStates.put("driveToFoundation", builder.create(new Vector3(9, -17, 0), 0, driveToFoundationTerminator, 0.3, "end")); //POWER: 0.25
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
                }
                if(timer != 0 && System.currentTimeMillis() > timer){
                    stateMachine.setActiveDriveState("driveToSkystones");
                    deactivateThis();
                }
                telemetry.setHeader("Time", timer - System.currentTimeMillis());
            }
        });
        final OrientationTerminator driveToSkystones = new OrientationTerminator(position, new Vector3(-10, -16, 90), 4, 5);
        driveStates.put("turnToSkystones", builder.turn(0.01, 90, "driveToSkystones", driveToSkystones));
        driveStates.put("driveToSkystones", builder.create(new Vector3(-10, -16, 90), 0, driveToSkystones, 1, "turnToSeeSkystones"));
        final OrientationTerminator turnToSeeSkystones = new OrientationTerminator(position, new Vector3(0, 0, 145), 0, 1);
        driveStates.put("turnToSeeSkystones", builder.turn(0.0075, 145, "strafeToSkystones", turnToSeeSkystones));
        final OrientationTerminator strafeToSkystones = new OrientationTerminator(position, new Vector3(-36, -16, 145), 4, 1);
        final PixyTerminator pixyTerminator = new PixyTerminator(position, new Vector3(-35, -16, 145), robot);
        final CombinedTerminator skystoneTerminator = new CombinedTerminator(position, new Vector3(-35, -16, 145), strafeToSkystones, pixyTerminator);
        states.put("pixyData", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(driveToSkystones.shouldTerminate(data) && timer == 0 && !(timer <= -1)) {
                    robot.intake(-1);
                    timer = 1;
                }
                if(System.currentTimeMillis() > timer && timer > 0){
                    robot.getPixy().queueData();
                    telemetry.setHeader("X", robot.getPixy().getY());
                    if(robot.getPixy().getY() > 110){
                        telemetry.setHeader("position", "c");
                    }else if(robot.getPixy().getY() < 110){
                        telemetry.setHeader("position", "l");
                    }else{
                        telemetry.setHeader("position", "r");
                    }
                }
                if(timer > 0 && turnToSeeSkystones.shouldTerminateRotation()){
                    robot.intake(0);
                }
                if(timer > 0 && skystoneTerminator.shouldTerminate(data) && stateMachine.driveStateIsActive("end")){
                    //skystoneCoords.set(new Vector3(position.getA() + (-6 * Math.cos(Math.toRadians(145))), position.getB() + (-6 * Math.sin(Math.toRadians(145))), 145));
                    sleep(1000);
                    skystoneCoords.set(position.getA()-4, -19, 145);
                    robot.intake(0.8);
                    timer = -1;
                    final OrientationTerminator intakeSkystone = new OrientationTerminator(position, skystoneCoords, 1, 1);
                    HashMap<String, DriveState> smallList = new HashMap<>();
                    smallList.put("intakeSkystones", builder.create(skystoneCoords, 0, intakeSkystone, 0.3, "end"));
                    stateMachine.appendDriveStates(smallList);
                    stateMachine.setActiveDriveState("intakeSkystones");
                }
            }
        });
        driveStates.put("strafeToSkystones", builder.create(new Vector3(-50, -16, 145), 0, skystoneTerminator, 0.15, "end"));
        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("init");
    }
}
