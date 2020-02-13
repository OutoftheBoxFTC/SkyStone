package opmode;

import android.media.MediaPlayer;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.R;

import java.util.HashMap;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.MecanumSystem;
import Motion.MotionSystem;
import Motion.Terminator.CombinedANDTerminator;
import Motion.Terminator.CombinedORTerminator;
import Motion.Terminator.OrientationTerminator;
import Motion.Terminator.RelativeOrientationTerminator;
import Motion.Terminator.SideLaserTerminator;
import Motion.Terminator.TimerTerminator;
import Motion.Terminator.TripwireTerminator;
import Motion.Terminator.VariedTripwireTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachine;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;
@Autonomous
public class NewNEWBlueAutonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    HashMap<String, Vector3> movements;
    long globTimer = 0;
    int skystonePos;
    public NewNEWBlueAutonomous() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        robot.disableDevice(Hardware.HardwareDevices.SIDE_LASERS);
        firstSkystone = Vector3.ZERO();
        movements = new HashMap<>();
        movements.put("moveToSkystone1", new Vector3(-14, 0, 40));
        movements.put("moveToSkystone2", new Vector3(-14, 2, 40));
        movements.put("moveToSkystone3", new Vector3(-14, 5, 40));
        movements.put("moveToIntakeBlock", new Vector3(-3, 5, 40));
        movements.put("driveToClearSkystones", new Vector3(-13, 2, 0));
        movements.put("driveToFoundation", new Vector3(-12, -40, 0));
        movements.put("turnAndDriveToFoundation", new Vector3(-18, -40, -90));
        movements.put("moveFoundationToScoringZone", new Vector3(-11, -30, 0));
        movements.put("driveBackToSkystones", new Vector3(-9, -20, 40));
        movements.put("driveBackToSkystones3", new Vector3(-9, -10, 40));
        movements.put("driveToSecondSkystone1", new Vector3(-19, -10, 50));
        movements.put("driveToSecondSkystone2", new Vector3(-14, -5, 50));
        movements.put("driveToSecondSkystone3", new Vector3(-14, -5, 50));
        movements.put("driveToFoundationV2", new Vector3(-13, -5, 0));
        movements.put("alignWithFoundationV2", new Vector3(-13, -27.5, 0));
        movements.put("driveToThirdStone", new Vector3(-12.5, -20, 0));
        movements.put("strafeToThirdStone", new Vector3(-19, -20, 0));
        movements.put("moveToAlignToFoundation", new Vector3(-16, 0, 0));
        movements.put("moveToFoundationV3", new Vector3(-14, -37.5, 0)); //change y to -40 for 3 stone, change to -30 for 4 stone
        movements.put("moveToFourthSkystone", new Vector3(-12.5, -10, 0));
        movements.put("strafeToAlignToFourthStone", new Vector3(-15, 0, 0));
        movements.put("grabFourthSkystone", new Vector3(-15, 30, 0));
        movements.put("alignWithFoundationV4", new Vector3(7.5, 0, 0));
        movements.put("park", new Vector3(0, 20, 0));
        final HashMap<String, Vector3> defaultTurns = new HashMap<>();
        defaultTurns.put("turnFoundation", new Vector3(0, 0, 0));
        defaultTurns.put("turnToIntake", new Vector3(0, 0, 0));
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
                telemetry.addData("Position", position);
            }
        });
        statemachine.appendLogicStates(nonManagedLogicStates);
        StateMachineManager init = new StateMachineManager(statemachine) {
            int y = 0;
            boolean[] map = new boolean[42];
            int[] byteMap = new int[42];
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    int counter = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(y > 207){
                            y = 0;
                            counter = 0;
                        }
                        byte[] tmp = robot.getPixy().getCoordinateColor(157, y);
                        byteMap[counter] = Math.abs(tmp[tmp.length-2] & 0xFF);
                        y += 5;
                        counter ++;
                        if(y == 210){
                            y = 207;
                            counter = 41;
                        }
                    }
                });
                logicStates.put("readout", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        for(int i = 0; i < map.length; i ++){
                            telemetry.addData("Position " + (i), byteMap[i]);
                        }
                    }
                });
                logicStates.put("result", new LogicState(statemachine) {
                    int pos1 = 0, pos2 = 0, pos3 = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        for(int i = 0; i < 10; i ++){
                            if(byteMap[i] < 255){
                                pos3 ++;
                            }
                        }
                        for(int i = 10; i < 20; i ++){
                            if(byteMap[i] < 255){
                                pos2 ++;
                            }
                        }
                        for(int i = 26; i < 32; i ++){
                            if(byteMap[i] < 255){
                                pos1 ++;
                            }
                        }
                        int max = Math.max(Math.max(pos1, pos2), pos3);
                        telemetry.addData("Max1", pos3);
                        telemetry.addData("Max2", pos2);
                        telemetry.addData("Max3", pos1);
                        telemetry.addData("Position", max == pos1 ? "3" : (max == pos2 ? "2" : "1"));
                        skystonePos = (max == pos1 ? 3 : (max == pos2 ? 2 : 1));
                        telemetry.addData("SkystonePos", skystonePos);
                        double conf1 = pos3;
                        double conf2 = pos2;
                        double conf3 = pos1;
                        double totConf = conf1 + conf2 + conf3;
                        telemetry.addData("Confidence 1", (conf1/totConf)*100);
                        telemetry.addData("Confidence 2", (conf2/totConf)*100);
                        telemetry.addData("Confidence 3", (conf3/totConf)*100);
                        pos1 = 0;
                        pos2 = 0;
                        pos3 = 0;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                telemetry.addData("Debug", "Got to this point");
                RobotLog.i("We are in the update loop");
                terminate = isStarted();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                stateMachine.activateLogic("Odometry");
                hardware.setIntakePowers(-1);
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                globTimer = System.currentTimeMillis();
            }
        };
        StateMachineManager moveToSkystones = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                if(skystonePos == 2){
                    driveState.put("main", system.driveToPointSlowdown(movements.get("moveToSkystone2"), 1));
                    terminator = new OrientationTerminator(position, movements.get("moveToSkystone2"), 4, 1);
                }else if(skystonePos == 3){
                    driveState.put("main", system.driveToPointSlowdown(movements.get("moveToSkystone3"), 1));
                    terminator = new OrientationTerminator(position, movements.get("moveToSkystone3"), 4, 1);
                }else{
                    driveState.put("main", system.driveToPointSlowdown(movements.get("moveToSkystone1"), 1));
                    terminator = new OrientationTerminator(position, movements.get("moveToSkystone1"), 4, 1);
                }
                logicStates.put("waitToIntake", new LogicState(statemachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 100;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > timer){
                            hardware.setIntakePowers(1);
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addData("SkystonePos", skystonePos);
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveToIntakeBlock = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator orientationTerminator;
            VariedTripwireTerminator tripwireTerminator;
            CombinedORTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveForward(movements.get("moveToIntakeBlock"), 0.5));
                logicStates.put("intake", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() < 11){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }
                    }
                });
                logicStates.put("init", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakePowers(1);
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        deactivateThis();
                    }
                });
                orientationTerminator = new RelativeOrientationTerminator(position, movements.get("moveToIntakeBlock"), 1.5);
                tripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 2.5, 5);
                terminator = new CombinedORTerminator(position, Vector3.ZERO(), orientationTerminator, tripwireTerminator);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager waitToClampOnToFirstBlock = new StateMachineManager(statemachine) {
            VariedTripwireTerminator variedTripwireTerminator;
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
                variedTripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 2, 5);
                exemptedLogicstates.put("latchOn", new LogicState(statemachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 500;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = variedTripwireTerminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager driveToClearSkystones = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointLinSlowdown(movements.get("driveToClearSkystones"), 0.5));
                terminator = new OrientationTerminator(position, movements.get("driveToClearSkystones"), 1, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                hardware.setIntakePowers(-1);
            }
        };
        StateMachineManager driveToFoundation = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(movements.get("driveToFoundation"), 1));
                terminator = new OrientationTerminator(position, movements.get("driveToFoundation"), 15, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager turnToLatchOn = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("turnAndDriveToFoundation"), 0.3));
                orientationTerminator = new OrientationTerminator(position, movements.get("turnAndDriveToFoundation"), 4, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_ON);
                hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
            }
        };
        StateMachineManager waitForLatchOn = new StateMachineManager(statemachine) {
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
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Lift", sensors.getLift());
                        hardware.setLiftMotors(1);
                        if(sensors.getLift() > 150){
                            hardware.setLiftMotors(0.2);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftMotors(0);
            }
        };
        StateMachineManager driveFoundationToScoreZone = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;

            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(movements.get("moveFoundationToScoringZone"), 1));
                terminator = new OrientationTerminator(position, movements.get("moveFoundationToScoringZone"), 3, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
            }
        };
        StateMachineManager turnFoundation = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.turn(defaultTurns.get("turnFoundation"), 0.5));
                orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnFoundation"), 1, 4);
                logicStates.put("fourBarBack", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminateRotation();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_OFF);
            }
        };
        StateMachineManager resetLift1 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftMotors(-0.3);
                        if(sensors.getLiftLimit()){
                            hardware.setLiftMotors(0);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftMotors(0);
            }
        };
        StateMachineManager driveBackToSkystones = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                if(skystonePos == 3){
                    driveState.put("main", system.driveToPointSlowdown(movements.get("driveBackToSkystones3"), 1));
                    orientationTerminator = new OrientationTerminator(position, movements.get("driveBackToSkystones3"), 4, 1);
                }else{
                    driveState.put("main", system.driveToPointSlowdown(movements.get("driveBackToSkystones"), 1));
                    orientationTerminator = new OrientationTerminator(position, movements.get("driveBackToSkystones"), 4, 1);
                }

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
            }
        };
        StateMachineManager turnToIntake = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.turn(defaultTurns.get("turnToIntake"), 0.35));
                orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnToIntake"), 4, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminateRotation();
                hardware.setIntakePowers(-1);
            }
        };
        StateMachineManager driveBackForSecondSkystone = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            VariedTripwireTerminator variedTripwireTerminator;
            CombinedORTerminator combinedORTerminator;
            @Override
            public void setup() {
                logicStates.put("intake", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() < 9){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }
                    }
                });
                telemetry.addData("Skystone", skystonePos);
                variedTripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 2.5, 5);
                if(skystonePos == 2){
                    driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone2"), 0.3));
                    terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone2"), 5, 0);
                }else if(skystonePos == 3){
                    driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone3"), 0.3));
                    terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone3"), 5, 0);
                }else{
                    driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone1"), 0.3));
                    terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone1"), 5, 0);
                }
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), new CombinedORTerminator(position, Vector3.ZERO(), terminator, variedTripwireTerminator), new TimerTerminator(position, Vector3.ZERO(), 2000));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
                telemetry.addData("Skystone", skystonePos);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(1);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
            }
        };
        StateMachineManager moveABitMoreToGetTheSecondSkystone = new StateMachineManager(statemachine) {
            CombinedORTerminator combinedORTerminator;
            @Override
            public void setup() {
                logicStates.put("intake", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() < 9){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }
                    }
                });
                driveState.put("main", new DriveState(statemachine) {
                    Vector4 powers = Vector4.ZERO();
                    long timer = 0;
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return powers;
                    }

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 1500;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            powers = Vector4.ZERO();
                            terminate = true;
                        }else{
                            powers = MecanumSystem.translate(new Vector3(0, -0.6, 0));
                        }
                    }
                });
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), new VariedTripwireTerminator(position, Vector3.ZERO(), 2.5, 5), new TimerTerminator(position, Vector3.ZERO(), 750));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager clampOnToSkystone = new StateMachineManager(statemachine) {
            VariedTripwireTerminator variedTripwireTerminator;
            @Override
            public void setup() {
                variedTripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 1, 5);
                driveState.put("stop", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("grabBlock", new LogicState(statemachine) {
                    long timer = 0;
                    boolean started = false;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(variedTripwireTerminator.shouldTerminate(sensors) && !started){
                            timer = System.currentTimeMillis() + 500;
                            started = true;
                        }
                        if(System.currentTimeMillis() > timer) {
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
            }
        };
        StateMachineManager clearBlocksV2 = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("driveToFoundationV2"), 1));
                terminator = new OrientationTerminator(position, movements.get("driveToFoundationV2"), 3, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager driveToFoundationSecondTime = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("alignWithFoundationV2"), 0.7));
                terminator = new OrientationTerminator(position, movements.get("alignWithFoundationV2"), 4, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
            }
        };
        StateMachineManager waitToStopMovingForSecondSkystone = new StateMachineManager(statemachine) {
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
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Lift", sensors.getLift());
                        hardware.setLiftMotors(1);
                        if(sensors.getLift() > 150){
                            hardware.setLiftMotors(0.2);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager unlatchSecondStone = new StateMachineManager(statemachine) {
            @Override
            public void onStart(SensorData sensors, HardwareData hardware) {
            }

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
                logicStates.put("main", new LogicState(statemachine) {
                    long timer = 0;
                    long timer2 = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 50;
                        timer2 = System.currentTimeMillis() + 100;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        }
                        if(System.currentTimeMillis() >= timer2){
                            hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setIntakePowers(-1);
            }
        };
        StateMachineManager resetLift2 = new StateMachineManager(statemachine) {
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
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftMotors(-0.3);
                        if(sensors.getLiftLimit()){
                            hardware.setLiftMotors(0);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftMotors(0);
            }
        };
        StateMachineManager driveToThirdStone = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("driveToThirdStone"), 1));
                orientationTerminator = new OrientationTerminator(position, movements.get("driveToThirdStone"), 4, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager rotateForThirdStone = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            VariedTripwireTerminator variedTripwireTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.turn(defaultTurns.get("turnToIntake"), 0.15));
                orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnToIntake"), 4, 4);
                variedTripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 10, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = (orientationTerminator.shouldTerminateRotation() || variedTripwireTerminator.shouldTerminate(sensors));
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setIntakePowers(1);
            }
        };
        StateMachineManager strafeForThirdStone = new StateMachineManager(statemachine) {
            CombinedORTerminator combinedORTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(movements.get("strafeToThirdStone"), 0.5));
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), new OrientationTerminator(position, movements.get("strafeToThirdStone"), 4, 4), new VariedTripwireTerminator(position, Vector3.ZERO(), 300, 3));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager grabThirdStone = new StateMachineManager(statemachine) {
            CombinedORTerminator combinedORTerminator;
            @Override
            public void setup() {
                driveState.put("main", new DriveState(statemachine) {
                    Vector4 powers = Vector4.ZERO();
                    long timer = 0;
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return powers;
                    }

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 2500;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            powers = Vector4.ZERO();
                            terminate = true;
                        }else{
                            powers = MecanumSystem.translate(new Vector3(0, -0.4, 0));
                        }
                    }
                });
                logicStates.put("closeIntake", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() < 11){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }
                    }
                });
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), new VariedTripwireTerminator(position, Vector3.ZERO(), 2.5, 5), new TimerTerminator(position, Vector3.ZERO(), 1500));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager waitToLatchOnToThirdBlock = new StateMachineManager(statemachine) {
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
                logicStates.put("latch", new LogicState(statemachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 500;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager moveToAlignWithFoundation = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("moveToAlignToFoundation"), 1));
                orientationTerminator = new OrientationTerminator(position, movements.get("moveToAlignToFoundation"), 4, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
            }
        };
        StateMachineManager moveToFoundationV3 = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("moveToFoundationV3"), 1));
                terminator = new OrientationTerminator(position, movements.get("moveToFoundationV3"), 4, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
            }
        };
        StateMachineManager raiseLift3 = new StateMachineManager(statemachine) {
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
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Lift", sensors.getLift());
                        hardware.setLiftMotors(1);
                        if(sensors.getLift() > 300){
                            hardware.setLiftMotors(0.2);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager unlatchThirdBlock = new StateMachineManager(statemachine) {
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
                logicStates.put("main", new LogicState(statemachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 500;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
            }
        };
        StateMachineManager resetLift3 = new StateMachineManager(statemachine) {
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
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftMotors(-0.3);
                        hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        if(sensors.getLiftLimit()){
                            hardware.setLiftMotors(0);
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftMotors(0);
            }
        };
        StateMachineManager waitForFourBarToGoDownForThirdSkystone = StateMachineManager.timer(500, statemachine);
        StateMachineManager driveToFourthSkystone = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("moveToFourthSkystone"), 1));
                orientationTerminator = new OrientationTerminator(position, movements.get("moveToFourthSkystone"), 4, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                orientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(1);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
            }
        };
        StateMachineManager strafeToAlignToFourthSkystone = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointSlowdown(movements.get("strafeToAlignToFourthStone"), 1));
                orientationTerminator = new OrientationTerminator(position, movements.get("strafeToAlignToFourthStone"), 4, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager grabFourthSkystone = new StateMachineManager(statemachine) {
            CombinedORTerminator combinedORTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(movements.get("grabFourthSkystone"), 0.4));
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), new OrientationTerminator(position, movements.get("grabFourthSkystone"), 4, 4), new VariedTripwireTerminator(position, Vector3.ZERO(), 2.5, 5));
                logicStates.put("closeIntake", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() < 11){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
            }
        };
        StateMachineManager strafeToAlignToFoundationV4 = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator relativeOrientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveForward(movements.get("alignWithFoundationV4"), 0.4));
                relativeOrientationTerminator = new RelativeOrientationTerminator(position, movements.get("alignWithFoundationV4"), 2);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                relativeOrientationTerminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager park = new StateMachineManager(statemachine) {
            RelativeOrientationTerminator relativeOrientationTerminator;
            @Override
            public void setup() {
                //player.start();
                driveState.put("main", (system.driveForward(movements.get("park"), 0.5)));
                relativeOrientationTerminator = new RelativeOrientationTerminator(position, movements.get("park"), 1);
                relativeOrientationTerminator.start();
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = relativeOrientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
            }
        };
        StateMachineManager end = new StateMachineManager(statemachine) {
            long timer = 0;
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
                timer = System.currentTimeMillis();
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addData("Global Timer", (timer-globTimer)/1000.0);
            }
        };
        stateMachineSwitcher.init(init, moveToSkystones, driveToIntakeBlock, waitToClampOnToFirstBlock, driveToClearSkystones, driveToFoundation, turnToLatchOn, waitForLatchOn, driveFoundationToScoreZone, turnFoundation, resetLift1, driveBackToSkystones, turnToIntake, driveBackForSecondSkystone, moveABitMoreToGetTheSecondSkystone, clampOnToSkystone, clearBlocksV2, driveToFoundationSecondTime, waitToStopMovingForSecondSkystone, unlatchSecondStone, resetLift2, driveToThirdStone, rotateForThirdStone, grabThirdStone, waitToLatchOnToThirdBlock, moveToAlignWithFoundation, moveToFoundationV3, raiseLift3, unlatchThirdBlock, resetLift3, waitForFourBarToGoDownForThirdSkystone, park, end);
    }
}
