package opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.HashMap;

import Debug.Registers;
import Hardware.*;
import Motion.MotionSystem;
import Motion.Terminator.CombinedTerminator;
import Motion.Terminator.OrientationTerminator;
import Motion.Terminator.PixyTerminator;
import Motion.Terminator.RelativeOrientationTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachine;
import State.StateMachineManager;
import State.StateMachineSwitcher;
import math.Vector3;
import math.Vector4;

@Autonomous
public class NewRedAutonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    Registers registers;
    public NewRedAutonomous() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        firstSkystone = Vector3.ZERO();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("moveToSkystones", "-10, 0, 0");
        defaults.put("scanSkystones", "0, -24, 0");
        defaults.put("moveToFoundation", "-10, 24, 90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("rotateToGrabSkystone", "0, 0, 80");
        registers = new Registers(defaults, defaultTurns);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem system = new MotionSystem(statemachine, odometer, position);
        HashMap<String, LogicState> nonManagedLogicStates = new HashMap<>();
        nonManagedLogicStates.put("Odometry", new LogicState(statemachine) {
            @Override
            public void init(SensorData sensors, HardwareData hardware){
                odometer.start(sensors);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                odometer.update(sensors);
                telemetry.addData("Position", position);
            }
        });
        statemachine.appendLogicStates(nonManagedLogicStates);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                terminate = isStarted();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                stateMachine.activateLogic("Odometry");
            }
        };
        StateMachineManager moveToSkystones = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(registers.getPoint("moveToSkystones"), 1));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("moveToSkystones"), 4);
            }
        };
        StateMachineManager driveToSkystone = new StateMachineManager(statemachine) {
            CombinedTerminator terminator;
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("scanSkystones"), 0.35));
                terminator = new CombinedTerminator(position, registers.getPoint("scanSkystones"), new OrientationTerminator(position, registers.getPoint("scanSkystones"), 4, 4), new PixyTerminator(position, registers.getPoint("scanSkystones")));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager rotateToGrabSkystone = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("main", system.turn(registers.getTurn("rotateToGrabSkystone"), 0.25));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminateRotation(position.getC(), registers.getTurn("rotateToGrabSkystone").getC(), 5);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                hardware.setIntakePowers(1);
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
            }
        };
        StateMachineManager moveForwardToGrabSkystone = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(-1.5, 0, 70), 0.5);
                driveState.put("drive", system.driveForward(new Vector3(-1.5, 0, 70), 0.35));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        terminator.start();
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }
            public void onStop(SensorData sensors, HardwareData hardware){
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
            }
        };
        StateMachineManager waitToIntake = new StateMachineManager(statemachine) {
            long timer = 0;
            @Override
            public void setup() {
                timer = System.currentTimeMillis() + 1000;
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = System.currentTimeMillis() > timer;
            }
        };
        StateMachineManager moveBackToGoToFoundation = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(2.5, 0, 0), 0.5);
                driveState.put("drive", system.driveForward(new Vector3(2.5, 0, 0), 0.35));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        terminator.start();
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }
            public void onStop(SensorData sensors, HardwareData hardware){
                hardware.setIntakePowers(0);
            }
        };
        StateMachineManager moveToFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(registers.getPoint("moveToFoundation"), 1));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("moveToFoundation"), 4);
            }
        };
        StateMachineManager end = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("main", new DriveState(statemachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.start(init, moveToSkystones, driveToSkystone, rotateToGrabSkystone, moveForwardToGrabSkystone, waitToIntake, moveBackToGoToFoundation, moveToFoundation, end);
    }
}
