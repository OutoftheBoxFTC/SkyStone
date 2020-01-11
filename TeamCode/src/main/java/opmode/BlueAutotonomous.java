package opmode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import Motion.Terminator.TripwireTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector2;
import math.Vector3;
import math.Vector4;
@Autonomous
public class BlueAutotonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    Registers registers;
    long fps;
    public BlueAutotonomous() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();

        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        firstSkystone = Vector3.ZERO();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("driveToFoundation", "4, -14, 0");
        defaults.put("driveBack", "4, -8, 0");
        defaults.put("strafeBeforeMovingToSkystones", "0, -3, 90"); //Relative
        defaults.put("driveToSeeSkystones", "-20, -11, 90");
        defaults.put("driveToSkystone", "-45, -9, 90");
        defaults.put("intakeSkystones", "0, -12, 185"); //Relative
        defaults.put("driveBackAfterIntake", "0, -14, 180"); //Relative
        defaults.put("driveToOuttake", "0, -11, 90");
        defaults.put("driveToSkystoneV2", "-55, -14, 90");
        defaults.put("driveToSeeSkystonesV2", "-30, -13, 90");
        defaults.put("strafeToLineUpToSkystone", "0, -3, 110"); //Relative
        defaults.put("intakeSkystonesV2", "-3, 0, 110"); //Relative
        defaults.put("driveBackAfterIntakingV2", "0, 6, 90"); //Relative
        defaults.put("driveToOuttakeV2", "-8, -9, 90");
        defaults.put("park", "-20, -12, 90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("turnToSkystones", "9, -1, 90");
        defaultTurns.put("turnToIntakeSkystone", "0, 0, 185");
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
                terminator = new RelativeOrientationTerminator(position, registers.getPoint("strafeBeforeMovingToSkystones"), 2);
                driveState.put("drive", system.driveForward(registers.getPoint("strafeBeforeMovingToSkystones"), 0.7));
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
                        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
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
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToSkystone"), 0.3));
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
                    firstSkystone.set(position.getA() - 9.5, -12, position.getC());
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
                        hardware.setLiftServo(HardwareConstants.LIFT_INTAKE.add(new Vector2(-0.01, -0.01)));
                        robot.disableDevice(Hardware.HardwareDevices.LEFT_PIXY);
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
            CombinedTerminator combinedTerminator;
            TripwireTerminator tripwire;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, registers.getPoint("intakeSkystones"), 2);
                tripwire = new TripwireTerminator(position, registers.getPoint("intakeSkystones"));
                driveState.put("drive", system.driveForward(registers.getPoint("intakeSkystones"), 0.35));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        terminator.start();
                        combinedTerminator = new CombinedTerminator(position, registers.getPoint("intakeSkystones"), terminator, tripwire);
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(tripwire.shouldTerminate(sensors)){
                            hardware.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedTerminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager closeIntake = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE_TELEOP);
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
            long timer = 0;
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
                        hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        terminate = true;
                        if(timer < 100) {
                            timer = System.currentTimeMillis() + 500;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
            }
        };
        final StateMachineManager driveBack = new StateMachineManager(statemachine) {
            Vector3 target;
            @Override
            public void setup() {
                target = registers.getPoint("driveBackAfterIntake");
                target.setA(position.getA());
                driveState.put("drive", system.driveToPoint(target, 0.45));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        hardware.setIntakePowers(0);
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, target, 3);
            }
        };
        StateMachineManager driveToOuttake = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPointSlowdown(registers.getPoint("driveToOuttake"), 1));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToOuttake"), 3);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
            }
        };
        StateMachineManager moveBackALittleMoreBeforeOuttake = new StateMachineManager(statemachine) {
            long timer = 0;
            @Override
            public void setup() {
                driveState.put("main", system.driveForward(new Vector3(12, 0, 90), 0.2));
                logicStates.put("timer", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 750;
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
                            hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
                            timer = System.currentTimeMillis() + 750;
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
                terminator = new RelativeOrientationTerminator(position, registers.getPoint("strafeToLineUpToSkystone"), 2);
                driveState.put("drive", system.driveForward(registers.getPoint("strafeToLineUpToSkystone"), 0.35));
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
                        hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
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
            CombinedTerminator combined;
            @Override
            public void setup() {
                terminator = new RelativeOrientationTerminator(position, registers.getPoint("intakeSkystonesV2"), 2);
                terminator.start();
                combined = new CombinedTerminator(Vector3.ZERO(), Vector3.ZERO(), terminator, new TripwireTerminator(Vector3.ZERO(), Vector3.ZERO()));
                driveState.put("drive", system.driveForward(registers.getPoint("intakeSkystonesV2"), 0.35));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combined.shouldTerminate(sensors);
            }
        };
        StateMachineManager closeIntakeV2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE_TELEOP);
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
                terminate = System.currentTimeMillis() > timer;
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
                        hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        terminate = true;
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
                terminator = new RelativeOrientationTerminator(position, registers.getPoint("driveBackAfterIntakingV2"), 2);
                driveState.put("drive", system.driveForward(registers.getPoint("driveBackAfterIntakingV2"), 0.35));
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        terminator.start();
                        hardware.setIntakePowers(-1);
                        hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
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
                driveState.put("drive", system.driveToPointSlowdown(registers.getPoint("driveToOuttakeV2"), 1));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToOuttakeV2"), 3);
            }
        };
        StateMachineManager moveBackALittleMoreBeforeOuttakeV2 = new StateMachineManager(statemachine) {
            long timer = 0;
            @Override
            public void setup() {
                driveState.put("main", system.driveForward(new Vector3(12, 0, 90), 0.2));
                logicStates.put("timer", new LogicState(statemachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        timer = System.currentTimeMillis() + 750;
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
                            hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
                            timer = System.currentTimeMillis() + 800;
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
        stateMachineSwitcher.start(init, driveToFoundation, latchOnToFoundation, driveFoundationBack, turnToSkystones, latchOffFoundation, strafeBeforeMovingToSkystones, driveToSeeSkystones, waitAfterSkystoneMovement, driveToSkystone, turnToIntakeSkystone, openIntake, intakeSkystone, closeIntake, secondIntakeMovement, lockIntake, driveBack, driveToOuttake, moveBackALittleMoreBeforeOuttake, outtakeSkystone, driveToSeeSkystonesV2, waitAfterSkystoneMovementV2, driveToSkystoneV2, strafeToLineUp, openIntakeV2, intakeSkystoneV2, closeIntakeV2, secondIntakeMovementV2, lockIntakeV2, driveBackV2, driveToOuttakeV2, moveBackALittleMoreBeforeOuttakeV2, outtakeSkystoneV2, park, end);
    }
}
