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
import state.Orientation;
import state.motion.MoveForward;
import state.motion.TurnCorrectionVector;

@TeleOp(name="BlueAutonomous")
public class BlueAutonomous extends BasicOpmode {
    public BlueAutonomous(){
        super(1, false);
    }
    Vector3 skystoneCoords =  Vector3.ZERO();
    OrientationTerminator intakeSkystone;
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
                    robot.intake(-1);
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

        final OrientationTerminator driveToFoundationTerminator = new OrientationTerminator(position, new Vector3(9, -17, 180), 2, 1);
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
        final OrientationTerminator driveFoundationBack = new OrientationTerminator(position, new Vector3(9, -1, 90), 3, 25);
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
        final OrientationTerminator driveToSkystones = new OrientationTerminator(position, new Vector3(-19, -14, 90), 8, 8);
        driveStates.put("turnToSkystones", builder.turn(0.015, 90, "driveToSkystones", driveToSkystones));
        driveStates.put("driveToSkystones", builder.create(new Vector3(-19, -12, 90), 0, driveToSkystones, 1, "turnToSeeSkystones"));
        final OrientationTerminator turnToSeeSkystones = new OrientationTerminator(position, new Vector3(0, 0, 160), 0, 4);
        driveStates.put("turnToSeeSkystones", builder.turn(0.001, 160, "strafeToSkystones", turnToSeeSkystones));
        final OrientationTerminator strafeToSkystones = new OrientationTerminator(position, new Vector3(-48, -16, 180), 1, 1);
        final PixyTerminator pixyTerminator = new PixyTerminator(position, new Vector3(-35, -14, 180), robot);
        final CombinedTerminator skystoneTerminator = new CombinedTerminator(position, new Vector3(-35, -14, 180), strafeToSkystones, pixyTerminator);
        states.put("pixyData", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(turnToSeeSkystones.shouldTerminateRotation() && timer == 0 && !(timer <= -1)) {
                    robot.variedIntake(0.4);
                    timer = 1500;
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
                    robot.variedIntake(0.4);
                }
                if(timer > 0 && skystoneTerminator.shouldTerminate(data) && stateMachine.driveStateIsActive("end")){
                    //skystoneCoords.set(new Vector3(position.getA() + (-6 * Math.cos(Math.toRadians(145))), position.getB() + (-6 * Math.sin(Math.toRadians(145))), 145));
                    skystoneCoords.set(position.getA()-2, position.getB(), 180);
                    robot.variedIntake(0.4);
                    timer = -1;
                    intakeSkystone = new OrientationTerminator(position, skystoneCoords, 1, 1);
                    HashMap<String, DriveState> smallList = new HashMap<>();
                    smallList.put("moveALittleMore", new MoveForward(stateMachine, drive, data, 0.2, "waitForSkystoneToIntake", TRANSLATION_FACTOR, 5, robot));
                    smallList.put("intakeSkystones", new MoveForward(stateMachine, drive, data, 0.2, "waitForSkystoneToIntake", TRANSLATION_FACTOR, 5, robot));
                    stateMachine.appendDriveStates(smallList);
                    stateMachine.activateLogic("waitForSkystone");
                    stateMachine.setActiveDriveState("moveALittleMore");
                    stateMachine.activateLogic("stallRobot");
                }
            }
        });
        driveStates.put("strafeToSkystones", builder.createSpecial(new Vector3(-48, -12, 180), 0, skystoneTerminator, 0.15, "end", 0, 1));
        final OrientationTerminator dumpSkystoneOnFoundation = new OrientationTerminator(position, new Vector3(-4, 0, 270), 5, 3);
        driveStates.put("turnToFoundationAgain", builder.turn(0.0075, 270, "dumpSkystoneOnFoundation", dumpSkystoneOnFoundation));
        driveStates.put("dumpSkystoneOnFoundation", builder.createSpecial(new Vector3(-4, 0, 270), 0, dumpSkystoneOnFoundation, 0.75, "end", 1, 1));
        states.put("stallRobot", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(dumpSkystoneOnFoundation.shouldTerminate(data) && timer == 0){
                    timer = System.currentTimeMillis() + 750;
                }
                if(System.currentTimeMillis() > timer && timer > 0){
                    robot.startDump();
                    robot.intake(0);
                    final OrientationTerminator driveBackToSkystones = new OrientationTerminator(position, new Vector3(skystoneCoords.getA() - 44, -3, skystoneCoords.getC()), 15, 8);
                    HashMap<String, DriveState> smallList = new HashMap<>();
                    smallList.put("turnBackToSkystones", builder.turn(1, 135, "driveToParkPos", driveBackToSkystones));
                    smallList.put("driveBackToSkystones", builder.createSpecial(new Vector3(skystoneCoords.getA() - 44, -3, 135), 0, driveBackToSkystones, 0.25, "intakeSkystonesV2", 1, 1));
                    final OrientationTerminator driveToPark = new OrientationTerminator(position, new Vector3((skystoneCoords.getA()-44)/2, 0, position.getC()), 8, 1);
                    smallList.put("driveToParkPos", builder.create(new Vector3((skystoneCoords.getA()-44)/2, 0, 135), 0, driveToPark, 0.5, "driveBackToSkystones"));
                    smallList.put("intakeSkystonesV2", new MoveForward(stateMachine, drive, data, 0.2, "end", TRANSLATION_FACTOR, 12, robot));
                    stateMachine.appendDriveStates(smallList);
                    stateMachine.setActiveDriveState("turnBackToSkystones");
                    telemetry.setHeader("skystone", skystoneCoords.getA());
                    deactivateThis();
                    //put next state here ig
                }
            }
        });
        driveStates.put("waitForSkystoneToIntake", new DriveState(stateMachine) {
            public void update(ReadData data){

            }
            @Override
            public Vector4 getWheelVelocities() {
                return Vector4.ZERO();
            }
        });
        states.put("waitForSkystone", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(timer == 0 && stateMachine.driveStateIsActive("waitForSkystoneToIntake")){
                    timer = System.currentTimeMillis() + 500;
                }
                if(System.currentTimeMillis() > timer && timer > 0){
                    HashMap<String, DriveState> smallList = new HashMap<>();
                    smallList.put("backUpALittle", new MoveForward(stateMachine, drive, data, -0.4, "turnToFoundationAgain", TRANSLATION_FACTOR, -4, robot));
                    robot.intake(0);
                    stateMachine.appendDriveStates(smallList);
                    stateMachine.setActiveDriveState("backUpALittle");
                    timer = -1;
                }
            }
        });
        states.put("intakingTheSkystonesV2", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {

            }
        });
        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("init");
    }
}
