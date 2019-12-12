package opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.HashMap;

import Debug.Connector;
import Debug.RecievePacket;
import Debug.Registers;
import Hardware.Hardware;
import Hardware.HardwareConstants;
import Hardware.HardwareData;
import Hardware.SensorData;
import Motion.MotionSystem;
import Motion.Terminator.CombinedTerminator;
import Motion.Terminator.OrientationTerminator;
import Motion.Terminator.PixyTerminator;
import Motion.Terminator.RelativeOrientationTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;
//-14
@Autonomous
public class BlueAutonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    Registers registers;
    public BlueAutonomous() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();

        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        firstSkystone = Vector3.ZERO();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("driveToFoundation", "4, -14, 0");
        defaults.put("driveBack", "4, -6, 0");
        defaults.put("driveToSeeSkystones", "-25, -10, 90");
        defaults.put("driveToSkystone", "-45, -9, 90");
        defaults.put("driveToOuttake", "0, -9, 90");
        defaults.put("driveToSkystoneV2", "-55, -12, 90");
        defaults.put("driveToSeeSkystonesV2", "-30, -10, 90");
        defaults.put("driveToOuttakeV2", "0, -10, 90");
        defaults.put("park", "-15, -12, 90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("turnToSkystones", "9, -1, 90");
        defaultTurns.put("turnToIntakeSkystone", "0, 0, 180");
        defaultTurns.put("turnToIntakeSkystoneV2", "0, 0, 145");
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
                for(byte b : sensors.getPixy()){
                    telemetry.addData("Pixy", b);
                }
            }
        });
        statemachine.appendLogicStates(nonManagedLogicStates);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                stateMachine.activateLogic("Odometry");
            }
        };
        StateMachineManager driveToFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToFoundation"), 0.5));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToFoundation"), 2);
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
                        timer = System.currentTimeMillis() + 1500;
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
            }
        };
        StateMachineManager driveFoundationBack = new StateMachineManager(statemachine) {
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
        StateMachineManager strafeBeforeMovingToSkystones = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(0, -4, 90), 2);
                driveState.put("drive", system.driveForward(new Vector3(0, -4, 90), 0.7));
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
        StateMachineManager waitAfterSkystoneMovement = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("stop", new DriveState(statemachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("wait", new LogicState(statemachine) {
                    long timer = 0;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 100;
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        terminate = System.currentTimeMillis() > timer;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveToSkystone = new StateMachineManager(statemachine) {
            CombinedTerminator terminator;
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToSkystone"), 0.2));
                terminator = new CombinedTerminator(position, registers.getPoint("driveToSkystone"), new OrientationTerminator(position, registers.getPoint("driveToSkystone"), 4, 4), new PixyTerminator(position, registers.getPoint("driveToSkystone")));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager turnToIntakeSkystone = new StateMachineManager(statemachine) {
            boolean bleh = true;
            @Override
            public void setup() {
                driveState.put("turn", system.turn(registers.getTurn("turnToIntakeSkystone"), 0.15));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                if(bleh){
                    firstSkystone.set(position.getA() - 10, -12, position.getC());
                    bleh = false;
                }
                terminate = OrientationTerminator.shouldTerminateRotation(position.getC(), 180, 5);
            }
        };
        StateMachineManager openIntake = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        hardware.setIntakePowers(1);
                        hardware.setLiftServo(0.12);
                        terminate = true;
                    }
                });
                driveState.put("drive", new DriveState(statemachine) {
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
        StateMachineManager intakeSkystone = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(0, -8, 180), 2);
                driveState.put("drive", system.driveForward(new Vector3(0, -8, 180), 0.35));
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
        };
        StateMachineManager closeIntake = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        hardware.setIntakePowers(1);
                        terminate = true;
                    }
                });
                driveState.put("drive", new DriveState(statemachine) {
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
        StateMachineManager secondIntakeMovement = new StateMachineManager(statemachine) {
            long timer = 0;
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 500;
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = (System.currentTimeMillis() > timer);
            }
        };
        StateMachineManager lockIntake = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("stop", new DriveState(statemachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("sequence", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftServo(HardwareConstants.LIFT_REST, HardwareConstants.LIFT_REST_OFFSET);
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                        terminate = true;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveBack = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(0, 7, 180), 2);
                driveState.put("drive", system.driveForward(new Vector3(0, 7, 180), 0.35));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        terminator.start();
                        hardware.setIntakePowers(0);
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
        StateMachineManager driveToOuttake = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToOuttake"), 0.6));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToOuttake"), 3);
            }
        };
        StateMachineManager outtakeSkystone = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("stop", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("sequence", new LogicState(statemachine) {
                    long timer = 0;
                    int state = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(state == 0){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT);
                            timer = System.currentTimeMillis() + 1500;
                            state = 1;
                        }
                        if(state == 1 && System.currentTimeMillis() > timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                            timer = System.currentTimeMillis() + 250;
                            state = 2;
                        }
                        if(state == 2 && System.currentTimeMillis() > timer){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            state = 3;
                            timer = System.currentTimeMillis() + 750;
                        }
                        if(state == 3 && System.currentTimeMillis() > timer){
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveToSeeSkystonesV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToSeeSkystonesV2"), 1, 1, 4));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToSeeSkystonesV2"), 4);
            }
        };
        StateMachineManager waitAfterSkystoneMovementV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("stop", new DriveState(statemachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("wait", new LogicState(statemachine) {
                    long timer = 0;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 100;
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        terminate = System.currentTimeMillis() > timer;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveToSkystoneV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(firstSkystone, 0.35));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, firstSkystone, 5);
            }
        };
        StateMachineManager strafeToLineUp = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(0, -5, 120), 2);
                driveState.put("drive", system.driveForward(new Vector3(0, -5, 120), 0.35));
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
        StateMachineManager openIntakeV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        hardware.setLiftServo(0.12);
                        hardware.setIntakePowers(1);
                        terminate = true;
                    }
                });
                driveState.put("drive", new DriveState(statemachine) {
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
        StateMachineManager intakeSkystoneV2 = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(-4.5, 0, 120), 2);
                driveState.put("drive", system.driveForward(new Vector3(-4.5, 0, 120), 0.35));
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
        };
        StateMachineManager closeIntakeV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        hardware.setIntakePowers(1);
                        terminate = true;
                    }
                });
                driveState.put("drive", new DriveState(statemachine) {
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
        StateMachineManager secondIntakeMovementV2 = new StateMachineManager(statemachine) {
            long timer = 0;
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 500;
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate =  System.currentTimeMillis() > timer;
            }
        };
        StateMachineManager lockIntakeV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("stop", new DriveState(statemachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("sequence", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveBackV2 = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator terminator;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, new Vector3(0, 6, 90), 2);
                driveState.put("drive", system.driveForward(new Vector3(0, 6, 90), 0.35));
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
        };
        StateMachineManager driveToOuttakeV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToOuttakeV2"), 0.6));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToOuttakeV2"), 3);
            }
        };
        StateMachineManager outtakeSkystoneV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("stop", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("sequence", new LogicState(statemachine) {
                    long timer = 0;
                    int state = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(state == 0){
                            hardware.setLiftServo(1);
                            timer = System.currentTimeMillis() + 250;
                            state = 1;
                        }
                        if(state == 1 && System.currentTimeMillis() > timer){
                            hardware.setIntakeLatch(0.35);
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                            timer = System.currentTimeMillis() + 250;
                            state = 2;
                        }
                        if(state == 2 && System.currentTimeMillis() > timer){
                            hardware.setLiftServo(0.02);
                            state = 3;
                            timer = System.currentTimeMillis() + 750;
                        }
                        if(state == 3 && System.currentTimeMillis() > timer){
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager park = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("park"), 0.6));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("park"), 5);
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
        StateMachineManager updateData = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("WaitForInput", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addLine("Press A to recieve data");
                        if(gamepad1.a){
                            statemachine.activateLogic("RecieveData");
                            deactivateThis();
                        }
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
                exemptedLogicstates.put("RecieveData", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        RecievePacket packet = Connector.getInstance().getDataFromMaster(2500);
                        if(packet != null){
                            ArrayList<String> positions = new ArrayList<>();
                            ArrayList<String> turns = new ArrayList<>();
                            turns.addAll(packet.turns);
                            positions.addAll(packet.positions);
                            registers.setPoints(positions);
                            registers.setTurns(turns);
                        }
                        statemachine.activateLogic("WaitToStart");
                        deactivateThis();
                    }
                });
                exemptedLogicstates.put("WaitToStart", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addLine("Press X to start Autonomous");
                        terminate = gamepad1.x;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.start(init, driveToFoundation, latchOnToFoundation, driveFoundationBack, turnToSkystones, latchOffFoundation, strafeBeforeMovingToSkystones, driveToSeeSkystones, waitAfterSkystoneMovement, driveToSkystone, turnToIntakeSkystone, openIntake, intakeSkystone, closeIntake, secondIntakeMovement, lockIntake, driveBack, driveToOuttake, outtakeSkystone, driveToSeeSkystonesV2, waitAfterSkystoneMovementV2, driveToSkystoneV2, strafeToLineUp, openIntakeV2, intakeSkystoneV2, closeIntakeV2, secondIntakeMovementV2, lockIntakeV2, driveBackV2, driveToOuttakeV2, outtakeSkystoneV2, park, end);
    }
}
