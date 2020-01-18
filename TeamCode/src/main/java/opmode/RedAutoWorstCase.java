package opmode;

import java.util.HashMap;

import Debug.Registers;
import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.MotionSystem;
import Motion.Terminator.OrientationTerminator;
import Motion.Terminator.RelativeOrientationTerminator;
import Motion.VelocitySystem;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;

public class RedAutoWorstCase extends BasicOpmode{
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    Registers registers;
    VelocitySystem vSystem;
    long fps;
    public RedAutoWorstCase() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();

        robot.enableDevice(Hardware.HardwareDevices.RIGHT_PIXY);
        firstSkystone = Vector3.ZERO();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("driveToFoundation", "4, -14, 0");
        defaults.put("driveBack", "4, -8, 0");
        defaults.put("driveToPark", "-10, -4, 90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("turnToSkystones", "9, -1, 90");
        registers = new Registers(defaults, defaultTurns);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        vSystem = new VelocitySystem();
        final MotionSystem system = new MotionSystem(statemachine, position, vSystem);
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
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                terminate = isStarted();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                stateMachine.activateLogic("Odometry");
                robot.disableDevice(Hardware.HardwareDevices.LEFT_PIXY);
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
            }
        };
        StateMachineManager driveToFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToFoundation"), 0.35));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToFoundation"), 2);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                hardware.setLatchServos(HardwareConstants.LATCH_ON);
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
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
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveBack"), 0.65));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveBack"), 2);
            }
        };
        StateMachineManager turnToSkystones = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.turn(registers.getTurn("turnToSkystones"), 0.5));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminateRotation(position.getC(), registers.getTurn("turnToSkystones").getC(), 5);
            }
        };
        StateMachineManager strafeBeforeMovingToSkystones = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(0, -3, 90), 2);
                driveState.put("drive", system.driveForward(new Vector3(0, -3, 90), 0.7));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        terminator.start();
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        terminate = terminator.shouldTerminate(sensors);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

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
        StateMachineManager driveToPark = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToPark"), 0.8));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToPark"), 3);
            }
        };
        StateMachineManager end = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                driveState.put("stop", new DriveState(statemachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }
        };
        stateMachineSwitcher.start(init, driveToFoundation, latchOnToFoundation, driveFoundationBack, turnToSkystones, latchOffFoundation, strafeBeforeMovingToSkystones, driveToPark, end);
    }
}
