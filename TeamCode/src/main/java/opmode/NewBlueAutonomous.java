package opmode;

import android.media.MediaPlayer;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.BatteryChecker;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.R;

import java.util.HashMap;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.CircleCorrectionVector;
import Motion.CorrectionVectorStrafeBiased;
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
public class NewBlueAutonomous extends BasicOpmode {
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
        movements.put("driveToClearSkystones", new Vector3(-11, 2, 0));
        movements.put("driveToFoundation", new Vector3(-12, -42, 0));
        movements.put("turnAndDriveToFoundation", new Vector3(-18, -42, -90));
        movements.put("moveFoundationToScoringZone", new Vector3(-12, -30, 15));
        movements.put("driveBackToSkystones", new Vector3(-12, -17, 0));
        movements.put("driveBackToSkystones3", new Vector3(-12, -15, 0));
        movements.put("driveToSecondSkystone1", new Vector3(-19, -12.5, 50));
        movements.put("driveToSecondSkystone2", new Vector3(-11, -5, 50));
        movements.put("driveToSecondSkystone3", new Vector3(-11, -2, 50));
        movements.put("driveToFoundationV2", new Vector3(-11, -5, 0));
        movements.put("alignWithFoundationV2", new Vector3(-11, -40, 0));
        movements.put("driveToThirdStone", new Vector3(-11, -10, 0));
        movements.put("driveToThirdStone3", new Vector3(-10, -12.5, 0));
        movements.put("moveToAlignToFoundation", new Vector3(-12, -5, 0));
        movements.put("moveToFoundationV3", new Vector3(-12, -40, 0)); //change y to -40 for 3 stone, change to -30 for 4 stone
        movements.put("moveToFourthSkystone", new Vector3(-12, -10, 0));
        movements.put("strafeToAlignToFourthStone", new Vector3(-15, 0, 0));
        movements.put("grabFourthSkystone", new Vector3(-15, 30, 0));
        movements.put("alignWithFoundationV4", new Vector3(7.5, 0, 0));
        movements.put("earlyPark", new Vector3(-12, -25, 0));
        movements.put("park", new Vector3(0, 10, 0));
        final HashMap<String, Vector3> defaultTurns = new HashMap<>();
        defaultTurns.put("turnFoundation", new Vector3(0, 0, 0));
        defaultTurns.put("turnToLatchOn", new Vector3(0, 0, 270));
        defaultTurns.put("turnToIntake", new Vector3(0, 0, 50));
        defaultTurns.put("turnToIntake3", new Vector3(0, 0, 70));
        defaultTurns.put("turnToZero", new Vector3(0, 0, 0));
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
                telemetry.addData("Voltage", hardware.getBattery());
                RobotLog.i("Voltage: " + hardware.getBattery());
                RobotLog.i("Time remaining: " + String.valueOf(30 - ((System.currentTimeMillis()-globTimer)/1000.0)));
            }
        });
        nonManagedLogicStates.put("clampOn", new LogicState(statemachine) {
            long timer = 0;

            @Override
            public void init(SensorData sensors, HardwareData hardware) {
                timer = System.currentTimeMillis() + 600;
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                if(System.currentTimeMillis() > (timer - 200)){
                    hardware.setLiftServo(HardwareConstants.LIFT_REST);
                }
                if(System.currentTimeMillis() > (timer)){
                    hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                }
                if(System.currentTimeMillis() > (timer + 50)){
                    deactivateThis();
                }
            }
        });
        statemachine.appendLogicStates(nonManagedLogicStates);
        final StateMachineManager end = new StateMachineManager(statemachine) {
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
        StateMachineManager init = new StateMachineManager(statemachine) {
            int y = 0;
            boolean[] map = new boolean[210];
            int[] byteMap = new int[210];
            @Override
            public void setup() {
                for(int i = 0; i < map.length; i ++){
                    byteMap[i] = 100;
                }
                logicStates.put("main", new LogicState(statemachine) {
                    int counter = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(y > 207){
                            y = 0;
                            counter = 0;
                        }
                        byte[] tmp = robot.getPixy().getCoordinateColor(140, y);
                        byteMap[counter] = Math.abs(tmp[tmp.length-2] & 0xFF);
                        y += 1;
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
                        for(int i = 0; i < 40; i ++){
                            if(byteMap[i] < 250){
                                pos3 ++;
                            }
                        }
                        for(int i = 40; i < 90; i ++){
                            if(byteMap[i] < 250){
                                pos2 ++;
                            }
                        }
                        for(int i = 90; i < 170; i ++){
                            if(byteMap[i] < 250){
                                pos1 ++;
                            }
                        }
                        int max = Math.max(Math.max(pos1, pos2), pos3);
                        telemetry.addData("Max1", pos3);
                        telemetry.addData("Max2", pos2);
                        telemetry.addData("Max3", pos1);
                        telemetry.addData("Position", max == pos1 ? "3" : (max == pos2 ? "2" : "1"));
                        skystonePos = (max == pos1 ? 3 : (max == pos2 ? 2 : 1));
                        if(pos1 == 0 && pos2 == 0 && pos3 == 0){
                            skystonePos = 3;
                        }
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
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                telemetry.addData("Debug", "Got to this point");
                terminate = isStarted();
                telemetry.addData("Voltage", hardware.getBattery());
                RobotLog.i("Voltage: " + hardware.getBattery());
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                stateMachine.activateLogic("Odometry");
                hardware.setIntakePowers(-1);
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                globTimer = System.currentTimeMillis();
            }
        };
        StateMachineManager waitAfterStart = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(stateMachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 100;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > (timer - 50)){
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        }
                        if(System.currentTimeMillis() > timer){
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
                hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
            }
        };
        final StateMachineManager moveToSkystones = new StateMachineManager(statemachine) {
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
                        timer = System.currentTimeMillis() + 20;
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
            TimerTerminator orientationTerminator;
            VariedTripwireTerminator tripwireTerminator;
            CombinedORTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveForward(movements.get("moveToIntakeBlock"), 0.5));
                logicStates.put("intake", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() < 9){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }
                    }
                });
                logicStates.put("init", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakePowers(1);
                        //hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        //hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        deactivateThis();
                    }
                });
                orientationTerminator = new TimerTerminator(position, Vector3.ZERO(), 1500);
                tripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 1.5, 5);
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
        final StateMachineManager strafeToClearSkystones = new StateMachineManager(statemachine) {
            OrientationTerminator relativeOrientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(new Vector3(-12, position.getB(), 0), 1));
                relativeOrientationTerminator = new OrientationTerminator(position, new Vector3(-12, position.getB(), 0), 2, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = relativeOrientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                statemachine.activateLogic("clampOn");
            }
        };
        StateMachineManager driveToClearSkystones = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            TimerTerminator timerTerminator;
            CombinedORTerminator combinedORTerminator;
            @Override
            public void setup() {
                driveState.put("main", new CorrectionVectorStrafeBiased(stateMachine, position, movements.get("driveToClearSkystones"), 0.4, Vector3.ZERO(), true));
                terminator = new OrientationTerminator(position, movements.get("driveToClearSkystones"), 3, 1);
                timerTerminator = new TimerTerminator(position, Vector3.ZERO(), 1500);
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), terminator, timerTerminator);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
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
                driveState.put("main", new CorrectionVectorStrafeBiased(stateMachine, position, movements.get("driveToFoundation"), 1, Vector3.ZERO(), true, 0.7));
                terminator = new OrientationTerminator(position, movements.get("driveToFoundation"), 10, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager turnABitMore = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.turn(defaultTurns.get("turnToLatchOn"), 0.4));
                orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnToLatchOn"), 4, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminateRotation();
            }
        };
        StateMachineManager turnToLatchOn = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(movements.get("turnAndDriveToFoundation"), 0.4));
                orientationTerminator = new OrientationTerminator(position, movements.get("turnAndDriveToFoundation"), 4, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_ON);
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
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 750;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Lift", sensors.getLift());
                        if(sensors.getLift() > 50){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT_AUTO);
                        }
                        hardware.setLiftMotors(1);
                        if(sensors.getLift() > 75){
                            hardware.setLiftMotors(0.2);
                            if(System.currentTimeMillis() >= timer) {
                                terminate = true;
                            }
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
                orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnFoundation"), 1, 8);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                terminate = orientationTerminator.shouldTerminateRotation();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager resetLift1 = new StateMachineManager(statemachine) {
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
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 150;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftMotors(-0.3);
                        if(System.currentTimeMillis() > timer){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        }
                        if(sensors.getLiftLimit()){
                            hardware.setLiftMotors(0);
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
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
                    driveState.put("main", system.driveToPointLinSlowdown(movements.get("driveBackToSkystones3"), 1));
                    orientationTerminator = new OrientationTerminator(position, movements.get("driveBackToSkystones3"), 4, 1);
                }else{
                    driveState.put("main", new CorrectionVectorStrafeBiased(stateMachine, position, movements.get("driveBackToSkystones"), 1, Vector3.ZERO(), true));
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
                hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
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
                    driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone2"), 1));
                    terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone2"), 5, 0);
                }else if(skystonePos == 3){
                    driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone3"), 1));
                    terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone3"), 5, 0);
                }else{
                    driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone1"), 1));
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
                        timer = System.currentTimeMillis() + 1000;
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
        StateMachineManager strafeToClearSkystonesV2 = new StateMachineManager(statemachine) {
            OrientationTerminator relativeOrientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(new Vector3(-12, position.getB(), 0), 0.7));
                relativeOrientationTerminator = new OrientationTerminator(position, new Vector3(-12, position.getB(), 0), 2, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = relativeOrientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                statemachine.activateLogic("clampOn");
                hardware.setIntakePowers(-1);
            }
        };
        StateMachineManager driveToFoundationSecondTime = StateMachineManager.timer(100, statemachine);
        StateMachineManager clearBlocksV2 = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", new CircleCorrectionVector(stateMachine, position, movements.get("alignWithFoundationV2"), movements.get("driveToFoundationV2"), 1, 6));
                terminator = new OrientationTerminator(position, movements.get("alignWithFoundationV2"), 10, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {

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
                        if(sensors.getLift() > 50){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT_AUTO);
                        }
                        if(sensors.getLift() > 250){
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
                        timer = System.currentTimeMillis() + 500;
                        timer2 = System.currentTimeMillis() + 1000;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        }
                        if(System.currentTimeMillis() >= timer2){
                            //hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
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
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
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
                if(skystonePos != 1) {
                    driveState.put("main", new CorrectionVectorStrafeBiased(stateMachine, position, movements.get("driveToThirdStone3"), 1, Vector3.ZERO()));
                    orientationTerminator = new OrientationTerminator(position, movements.get("driveToThirdStone3"), 6, 1);
                }else{
                    driveState.put("main", new CorrectionVectorStrafeBiased(stateMachine, position, movements.get("driveToThirdStone"), 1, Vector3.ZERO()));
                    orientationTerminator = new OrientationTerminator(position, movements.get("driveToThirdStone"), 6, 1);
                }
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
                hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
            }
        };
        StateMachineManager rotateForThirdStone = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            VariedTripwireTerminator variedTripwireTerminator;
            @Override
            public void setup() {
                if(skystonePos != 3) {
                    driveState.put("main", system.turn(defaultTurns.get("turnToIntake"), 0.15));
                    orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnToIntake"), 4, 4);
                }else{
                    driveState.put("main", system.turn(defaultTurns.get("turnToIntake3"), 0.15));
                    orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnToIntake3"), 4, 4);
                }
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
        StateMachineManager strafeToClearSkystones3 = new StateMachineManager(statemachine) {
            OrientationTerminator relativeOrientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(new Vector3(-12, position.getB(), 0), 0.7));
                relativeOrientationTerminator = new OrientationTerminator(position, new Vector3(-12, position.getB(), 0), 4, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = relativeOrientationTerminator.shouldTerminate(sensors);
                statemachine.activateLogic("clampOn");
            }
        };
        final StateMachineManager earlyPark = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPointLinSlowdown(movements.get("earlyPark"), 0.5));
                orientationTerminator = new OrientationTerminator(position, movements.get("earlyPark"), 5, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                stateMachineSwitcher.setActive(end, sensors, hardware);
            }
        };
        StateMachineManager turnToZero = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            OrientationTerminator testTerminatorl;
            @Override
            public void setup() {
                driveState.put("main", system.turn(defaultTurns.get("turnToZero"), 0.35));
                orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnToZero"), 2, 10);
                testTerminatorl = new OrientationTerminator(position, defaultTurns.get("turnToZero"), 2, 20);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminateRotation();
                if(testTerminatorl.shouldTerminateRotation()){
                    if(sensors.getIntakeTripwire() > 15){
                        //stateMachineSwitcher.setActive(earlyPark, sensors, hardware);
                    }
                }
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
            }
        };
        StateMachineManager moveToAlignWithFoundation = StateMachineManager.timer(500, statemachine);
        StateMachineManager moveToFoundationV3 = new StateMachineManager(statemachine) {
            CombinedORTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", new CircleCorrectionVector(stateMachine, position, movements.get("moveToFoundationV3"), movements.get("moveToAlignToFoundation"), 1, 6));
                terminator = new CombinedORTerminator(position, Vector3.ZERO(), new OrientationTerminator(position, movements.get("moveToFoundationV3"), 8, 4), new TimerTerminator(position, Vector3.ZERO(), 2500));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(-1);
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
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
                        if(sensors.getLift() > 50){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT_AUTO);
                        }
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
                driveState.put("main", (system.driveForward(movements.get("park"), 1)));
                relativeOrientationTerminator = new RelativeOrientationTerminator(position, movements.get("park"), 3);
                relativeOrientationTerminator.start();
                logicStates.put("timer", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(((System.currentTimeMillis() - globTimer)/1000) < 0.25){
                            terminate = true;
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = relativeOrientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
            }
        };
        stateMachineSwitcher.init(init, waitAfterStart, moveToSkystones, driveToIntakeBlock, strafeToClearSkystones, driveToClearSkystones, driveToFoundation, turnABitMore, turnToLatchOn, waitForLatchOn, driveFoundationToScoreZone, turnFoundation, resetLift1, driveBackToSkystones, turnToIntake, driveBackForSecondSkystone, moveABitMoreToGetTheSecondSkystone, strafeToClearSkystonesV2, driveToFoundationSecondTime, clearBlocksV2, waitToStopMovingForSecondSkystone, unlatchSecondStone, resetLift2, driveToThirdStone, rotateForThirdStone, grabThirdStone, strafeToClearSkystones3, turnToZero, moveToAlignWithFoundation, moveToFoundationV3, raiseLift3, unlatchThirdBlock, resetLift3, waitForFourBarToGoDownForThirdSkystone, park, end, earlyPark);
    }
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    HashMap<String, Vector3> movements;
    long globTimer = 0;
    int skystonePos;

    public NewBlueAutonomous() {
        super(1);
    }
}
