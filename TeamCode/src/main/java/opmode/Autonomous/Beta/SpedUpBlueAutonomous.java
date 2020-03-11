package opmode.Autonomous.Beta;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.HashMap;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.CircleCorrectionVector;
import Motion.CorrectionVectorStrafeBiased;
import Motion.CurveBuilder;
import Motion.MecanumSystem;
import Motion.MotionSystem;
import Motion.Terminator.CombinedORTerminator;
import Motion.Terminator.OrientationTerminator;
import Motion.Terminator.RelativeOrientationTerminator;
import Motion.Terminator.TimerTerminator;
import Motion.Terminator.VariedTripwireTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector2;
import math.Vector3;
import math.Vector4;
import opmode.BasicOpmode;

@Autonomous
public class SpedUpBlueAutonomous extends BasicOpmode {
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
        movements.put("driveToBridge", new Vector3(-12, -20, 0));
        movements.put("driveToFoundation", new Vector3(-12, -40, -90));
        movements.put("turnAndDriveToFoundation", new Vector3(-18, -42, -90));
        movements.put("moveFoundationToScoringZone", new Vector3(-12, -30, 15));
        movements.put("driveBackToSkystones", new Vector3(-12, -15, 0));
        movements.put("driveBackToSkystones3", new Vector3(-12, -15, 0));
        movements.put("driveToSecondSkystone1", new Vector3(-19, -12.5, 50));
        movements.put("driveToSecondSkystone2", new Vector3(-11, -6, 50));
        movements.put("driveToSecondSkystone3", new Vector3(-11, -2, 50));
        movements.put("driveToFoundationV2", new Vector3(-11, -5, 0));
        movements.put("driveToBridge2", new Vector3(-11, -20, 0));
        movements.put("alignWithFoundationV2", new Vector3(-11, -40, 0));
        movements.put("driveToThirdStone", new Vector3(-11, -10, 0));
        movements.put("driveToThirdStone3", new Vector3(-10, -20, 0));
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
        final HashMap<String, LogicState> nonManagedLogicStates = new HashMap<>();
        final CurveBuilder curveBuilder = new CurveBuilder(statemachine, position);
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
                telemetry.addData("tripwire", sensors.getIntakeTripwire());
                RobotLog.i("Voltage: " + hardware.getBattery());
                RobotLog.i("Time remaining: " + String.valueOf(30 - ((System.currentTimeMillis()-globTimer)/1000.0)));
            }
        });
        nonManagedLogicStates.put("clampOn", new LogicState(statemachine) {
            long timer = 0;

            @Override
            public void init(SensorData sensors, HardwareData hardware) {
                timer = System.currentTimeMillis() + 100;
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                if(System.currentTimeMillis() > (timer - 200)){
                    hardware.setLiftServo(HardwareConstants.LIFT_REST);
                }
                if(System.currentTimeMillis() >= (timer)){
                    hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                }
                if(System.currentTimeMillis() > (timer+10)){
                    deactivateThis();
                }
            }
        });
        nonManagedLogicStates.put("liftScore", new LogicState(statemachine) {
            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                if(sensors.getLift() < 100){
                    hardware.setLiftMotors(0.5);
                }else{
                    hardware.setLiftMotors(0.2);
                    deactivateThis();
                }
                if(sensors.getLift() > 50){
                    hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
                }
            }
        });
        nonManagedLogicStates.put("scoreBlock", new LogicState(statemachine) {
            long timer = 0;
            boolean unlatched = false;
            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                if(timer == 0) {
                    if (sensors.getLift() < 60) {
                        hardware.setLiftMotors(0.75);
                    } else {
                        hardware.setLiftMotors(0.2);
                        timer = System.currentTimeMillis() + 400;
                    }
                    if (sensors.getLift() > 25) {
                        hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
                    }
                }
                if(System.currentTimeMillis() > timer && !unlatched && timer != 0){
                    hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                    unlatched = true;
                    timer = System.currentTimeMillis() + 150;
                }
                if(System.currentTimeMillis() > timer && unlatched && timer != 0){
                    hardware.setLiftServo(HardwareConstants.LIFT_REST);
                    if(!sensors.getLiftLimit()){
                        hardware.setLiftMotors(-0.4);
                    }else{
                        hardware.setLiftMotors(0);
                        deactivateThis();
                    }
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
                    byteMap[i] = 255;
                }
                logicStates.put("main", new LogicState(statemachine) {
                    int counter = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        robot.disableDevice(Hardware.HardwareDevices.INTAKE_TRIPWIRE);
                    }

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
                            //telemetry.addData("Position " + (i), byteMap[i]);
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
                robot.disableDevice(Hardware.HardwareDevices.LEFT_PIXY);
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
                    driveState.put("main", system.driveToPoint(movements.get("moveToSkystone2"), 1));
                    terminator = new OrientationTerminator(position, movements.get("moveToSkystone2"), 4, 1);
                }else if(skystonePos == 3){
                    driveState.put("main", system.driveToPoint(movements.get("moveToSkystone3"), 1));
                    terminator = new OrientationTerminator(position, movements.get("moveToSkystone3"), 4, 1);
                }else{
                    driveState.put("main", system.driveToPoint(movements.get("moveToSkystone1"), 1));
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
                            hardware.setIntakePowers(0.6);
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
                robot.enableDevice(Hardware.HardwareDevices.INTAKE_TRIPWIRE);
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
                        if(sensors.getIntakeTripwire() < 10){
                            hardware.setIntakePowers(1);
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }
                        if(sensors.getLift() < 10){
                            hardware.setLiftMotors(0.5);
                        }else{
                            hardware.setLiftMotors(0.2);
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
                orientationTerminator = new TimerTerminator(position, Vector3.ZERO(), 1000);
                tripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 5.5, 5);
                terminator = new CombinedORTerminator(position, Vector3.ZERO(), orientationTerminator, tripwireTerminator);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager driveToFoundation = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", curveBuilder.newCurve()
                                .setSpline(new Vector2(-12, position.getB()))
                                .setSpline(new Vector2(-12, -35))
                                .setSpeed(1)
                                .setMinPower(0.4)
                                .complete());
                logicStates.put("timer", new LogicState(stateMachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 300;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(!sensors.getLiftLimit()){
                            hardware.setLiftMotors(-0.2);
                        }else{
                            hardware.setLiftMotors(0);
                        }
                        if(System.currentTimeMillis() > timer){
                            hardware.setIntakePowers(-0.5);
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                        }
                    }
                });
                terminator = new OrientationTerminator(position, movements.get("driveToFoundation"), 10, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_MID);
            }
        };
        StateMachineManager turnABitMore = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.turn(defaultTurns.get("turnToLatchOn"), 0.6));
                orientationTerminator = new OrientationTerminator(position, defaultTurns.get("turnToLatchOn"), 4, 4);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminateRotation();
                hardware.setLatchServos(HardwareConstants.LATCH_MID);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_MID);
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
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                stateMachine.activateLogic("scoreBlock");
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
                        timer = System.currentTimeMillis() + 1000; //You can't do anything right
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
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);

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
                        hardware.setLiftMotors(-0.5);
                        if(System.currentTimeMillis() > timer){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        }
                        if(sensors.getLiftLimit()){
                            hardware.setLiftMotors(0);
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            robot.enableDevice(Hardware.HardwareDevices.INTAKE_TRIPWIRE);
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
                hardware.setIntakePowers(0.6);
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
                if(stateMachine.logicStateActive("scoreBlock")){
                    stateMachine.deactivateLogic("scoreBlock");
                }
                robot.enableDevice(Hardware.HardwareDevices.INTAKE_TRIPWIRE);
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

                    }
                });
                telemetry.addData("Skystone", skystonePos);
                if(skystonePos == 2){
                    driveState.put("main", curveBuilder.newCurve()
                            .setSpline(new Vector2(-12, -15))
                            .setSpline(new Vector2(-11, -6))
                            .complete());
                    terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone2"), 5, 0);
                }else if(skystonePos == 3){
                    driveState.put("main", curveBuilder.newCurve()
                            .setSpline(new Vector2(-12, -13))
                            .setSpline(new Vector2(-12, -6))
                            .setSpline(new Vector2(-15, -6))
                            .setAngle(0)
                            .setTurnStartDistance(10)
                            .setInitialAngle(0)
                            .complete());
                    terminator = new OrientationTerminator(position, new Vector3(-15, -12.5, 0), 5, 0);
                }else{
                    driveState.put("main", curveBuilder.newCurve()
                            .setSpline(new Vector2(-12, -13))
                            .setSpline(new Vector2(-12, -8))
                            .setSpline(new Vector2(-20, -8))
                            .setAngle(40)
                            .setTurnStartDistance(15)
                            .setInitialAngle(0)
                            .complete());
                    terminator = new OrientationTerminator(position, new Vector3(-15, -8, 0), 2, 0);
                }
                logicStates.put("lift", new LogicState(stateMachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            if(stateMachine.logicStateActive("scoreBlock")){
                                stateMachine.deactivateLogic("scoreBlock");
                            }
                            if(sensors.getLift() < 20){
                                hardware.setLiftMotors(0.5);
                            }else{
                                hardware.setLiftMotors(0.3);
                            }
                        }
                    }
                });
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), terminator, new TimerTerminator(position, Vector3.ZERO(), 2000));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
                telemetry.addData("Skystone", skystonePos);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(0.8);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
            }
        };
        StateMachineManager moveABitMoreToGetTheSecondSkystone = new StateMachineManager(statemachine) {
            CombinedORTerminator combinedORTerminator;
            TimerTerminator timerTerminator;
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
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), new VariedTripwireTerminator(position, Vector3.ZERO(), 8, 1), new TimerTerminator(position, Vector3.ZERO(), 750));
                timerTerminator = new TimerTerminator(position, Vector3.ZERO(), 750);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
            }
        };
        StateMachineManager driveWhileIntakingBlock = new StateMachineManager(statemachine) {
            OrientationTerminator terminator, orientationTerminator, latchOnTerminator;
            @Override
            public void setup() {
                driveState.put("main", curveBuilder.newCurve()
                        .setSpline(new Vector2(-10, position.getB()))
                        .setSpline(new Vector2(-10, -40))
                        .setRadius(1)
                        .setSpeed(0.4)
                        .complete());
                logicStates.put("lift", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() < 2) {
                            if (!sensors.getLiftLimit()) {
                                hardware.setLiftMotors(-0.5);
                            } else {
                                hardware.setLiftMotors(0);
                                terminate = true;
                            }
                        }
                        if(sensors.getLift() < 20){
                            hardware.setLiftMotors(0.5);
                        }else{
                            hardware.setLiftMotors(0.3);
                        }
                    }
                });
                terminator = new OrientationTerminator(position, new Vector3(-11, -30, 0), 8, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = (terminator.shouldTerminate(sensors) || (sensors.getIntakeTripwire() < 3));
                hardware.setIntakePowers(1);

            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager driveToFoundationSecondTime = new StateMachineManager(statemachine) {
            OrientationTerminator terminator, orientationTerminator, latchOnTerminator;
            @Override
            public void setup() {
                driveState.put("main", curveBuilder.newCurve()
                        .setSpline(new Vector2(-10, position.getB()))
                        .setSpline(new Vector2(-10, -40))
                        .setRadius(1)
                        .setMinPower(0.2)
                        .complete());
                terminator = new OrientationTerminator(position, new Vector3(-11, -30, 0), 8, 1);
                orientationTerminator = new OrientationTerminator(position, new Vector3(-11, -30, 0), 15, 1);
                latchOnTerminator = new OrientationTerminator(position, new Vector3(-11, -30, 0), 20, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
                if(latchOnTerminator.shouldTerminate(sensors)){
                    hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                }
                hardware.setIntakePowers(1);

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
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 100;
                        //hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > timer) {
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
                logicStates.put("lift", new LogicState(stateMachine) {
                    long timer = 0;
                    boolean unlatched = false;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(timer == 0) {
                            if (sensors.getLift() < 200) {
                                hardware.setLiftMotors(0.75);
                            } else {
                                hardware.setLiftMotors(0.2);
                                timer = System.currentTimeMillis() + 150;
                            }
                            if (sensors.getLift() > 100) {
                                hardware.setLiftServo(HardwareConstants.LIFT_OUT_AUTO);
                            }
                        }
                        if(System.currentTimeMillis() > timer && !unlatched && timer != 0){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                            unlatched = true;
                            timer = System.currentTimeMillis() + 150;
                        }
                        if(System.currentTimeMillis() > timer && unlatched && timer != 0){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            if(!sensors.getLiftLimit()){
                                hardware.setLiftMotors(-0.5);
                            }else{
                                hardware.setLiftMotors(0);
                                terminate = true;
                            }
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                //terminate = (hardware.getIntakeLatch() == HardwareConstants.INTAKE_LATCH_OFF);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
                //hardware.setIntakePowers(-1);
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
                        if(sensors.getLiftLimit()){
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
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setIntakePowers(0.6);
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
                    driveState.put("main", curveBuilder.newCurve()
                            .setSpline(new Vector2(-12, -15))
                            .setSpline(new Vector2(-12, -5))
                            .setSpline(new Vector2(-20, 5))
                            .setTurnStartDistance(15)
                            .setAngle(50)
                            .complete());
                    orientationTerminator = new OrientationTerminator(position, new Vector3(-20, 5, 0), 2, 1);
                }
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
                hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(1);
                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
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
                            powers = MecanumSystem.translate(new Vector3(0, -0.6, 0));
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
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), new VariedTripwireTerminator(position, Vector3.ZERO(), 2.5, 5), new TimerTerminator(position, Vector3.ZERO(), 1000));
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
        StateMachineManager moveToFoundationV3 = new StateMachineManager(statemachine) {
            CombinedORTerminator terminator;
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", curveBuilder.newCurve()
                        .setSpline(new Vector2(-10, -15))
                        .setSpline(new Vector2(-10, -40))
                        .setRadius(0.5)
                        .setMinPower(0.3)
                        .setRadius(3)
                        .complete());
                terminator = new CombinedORTerminator(position, Vector3.ZERO(), new OrientationTerminator(position, new Vector3(-10, -40, 0), 2, 4), new TimerTerminator(position, Vector3.ZERO(), 15000));
                orientationTerminator = new OrientationTerminator(position, new Vector3(-10, -40, 0), 15, 2);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(0);
                terminate = terminator.shouldTerminate(sensors);
                if(orientationTerminator.shouldTerminate(sensors)){
                    hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                }
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
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                hardware.setIntakePowers(0.6);
            }
        };
        StateMachineManager waitForFourBarToGoDownForThirdSkystone = StateMachineManager.timer(500, statemachine);
        StateMachineManager driveToFourthSkystone = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", curveBuilder.newCurve()
                        .setSpline(new Vector2(-12, -15))
                        .setSpline(new Vector2(-12, 5))
                        .setSpline(new Vector2(-20, 10))
                        .setTurnStartDistance(15)
                        .setAngle(50)
                        .complete());
                orientationTerminator = new OrientationTerminator(position, new Vector3(-20, 10, 0), 2, 4);
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
        stateMachineSwitcher.init(init, waitAfterStart, moveToSkystones, driveToIntakeBlock, driveToFoundation, turnABitMore, turnToLatchOn, waitForLatchOn, driveFoundationToScoreZone, turnFoundation, resetLift1, driveBackForSecondSkystone, moveABitMoreToGetTheSecondSkystone, driveWhileIntakingBlock, driveToFoundationSecondTime, waitToStopMovingForSecondSkystone, unlatchSecondStone, resetLift2, driveToThirdStone, grabThirdStone, moveToFoundationV3, raiseLift3, unlatchThirdBlock, resetLift3, waitForFourBarToGoDownForThirdSkystone, driveToFourthSkystone, park, end);
    }
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    HashMap<String, Vector3> movements;
    long globTimer = 0;
    int skystonePos;

    public SpedUpBlueAutonomous() {
        super(1);
    }
}
