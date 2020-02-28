package opmode.Autonomous.Competition;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.HashMap;

import Debug.Connector;
import Debug.RecievePacket;
import Debug.Registers;
import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.MotionSystem;
import Motion.Terminator.CombinedORTerminator;
import Motion.Terminator.OrientationTerminator;
import Motion.Terminator.PixyTerminator;
import Motion.Terminator.RelativeOrientationTerminator;
import Motion.Terminator.TimerTerminator;
import Motion.Terminator.TripwireTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import State.VelocityDriveState;
import math.Vector2;
import math.Vector3;
import math.Vector4;
import opmode.BasicOpmode;

@Autonomous
public class RedFoundationAndPark extends BasicOpmode {
    private SimpleOdometer odometer;
    private Vector3 position, velocity, firstSkystone;
    private Registers registers;
    long fps;
    public RedFoundationAndPark() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();

        robot.enableDevice(Hardware.HardwareDevices.RIGHT_PIXY);
        firstSkystone = Vector3.ZERO();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("driveToFoundation", "-4, -13, 0");
        defaults.put("driveBack", "2, -6, 0");
        defaults.put("strafeBeforeMovingToSkystones", "0, 2, -90"); //Relative
        defaults.put("driveToSeeSkystones", "10, 2, -90");
        defaults.put("park", "17, -12, -90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("turnToSkystones", "-9, -1, -90");
        defaultTurns.put("turnToIntakeSkystone", "0, 0, -180");
        defaultTurns.put("turnToIntakeSkystoneV2", "0, 0, -145");
        registers = new Registers(defaults, defaultTurns);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem system = new MotionSystem(statemachine, position, velocity);
        HashMap<String, LogicState> nonManagedLogicStates = new HashMap<>();
        nonManagedLogicStates.put("Odometry", new LogicState(statemachine) {
            @Override
            public void init(SensorData sensors, HardwareData hardware){
                odometer.start(sensors);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                odometer.update(sensors);
                telemetry.addData("FPS", 1000/(System.currentTimeMillis() - fps));
                fps = System.currentTimeMillis();
                hardware.setCapstoneLatch(HardwareConstants.CAPSTONE_LATCH_OFF);
            }
        });
        statemachine.appendLogicStates(nonManagedLogicStates);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                hardware.setCapstoneLatch(HardwareConstants.CAPSTONE_LATCH_OFF);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
                terminate = isStarted();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                stateMachine.activateLogic("Odometry");
                robot.disableDevice(Hardware.HardwareDevices.LEFT_PIXY);
            }
        };
        StateMachineManager driveToFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToFoundation"), 0.4));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToFoundation"), 3);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                hardware.setLatchServos(HardwareConstants.LATCH_ON);
            }
        };
        StateMachineManager latchOnToFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("latch", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLatchServos(HardwareConstants.LATCH_ON);
                    }
                });
                driveState.put("stop", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("wait", new LogicState(stateMachine) {
                    long timer = 0;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 500;
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > timer){
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(0);
                hardware.setLatchServos(HardwareConstants.LATCH_ON);
            }
        };
        final StateMachineManager driveFoundationBack = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveBack"), 1));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveBack"), 3);
            }
        };
        StateMachineManager turnToSkystones = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.turn(registers.getTurn("turnToSkystones"), 0.75));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminateRotation(position.getC(), registers.getTurn("turnToSkystones").getC(), 5);
            }
        };
        StateMachineManager strafeBeforeMovingToSkystones = new StateMachineManager(statemachine) {
            TimerTerminator terminator;
            @Override
            public void setup() {
                terminator = new TimerTerminator(position, Vector3.ZERO(), 1000);
                driveState.put("drive", new VelocityDriveState(stateMachine) {
                    @Override
                    public Vector3 getVelocities() {
                        return new Vector3(-0.4, 0, 0);
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
        };
        StateMachineManager latchOffFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("latch", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                    }
                });
                driveState.put("stop", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("wait", new LogicState(stateMachine) {
                    long timer = 0;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 500;
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > timer){
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveToSeeSkystones = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToSeeSkystones"), 0.8));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToSeeSkystones"), 3);
            }
        };
        StateMachineManager end = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("stop", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        terminate = false;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.init(init, driveToFoundation, latchOnToFoundation, driveFoundationBack, turnToSkystones, latchOffFoundation, strafeBeforeMovingToSkystones, driveToSeeSkystones, end);
    }
}
