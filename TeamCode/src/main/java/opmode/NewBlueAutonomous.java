package opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.HashMap;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.MotionSystem;
import Motion.Terminator.CombinedANDTerminator;
import Motion.Terminator.CombinedORTerminator;
import Motion.Terminator.OrientationTerminator;
import Motion.Terminator.RelativeOrientationTerminator;
import Motion.Terminator.SideLaserTerminator;
import Motion.Terminator.TripwireTerminator;
import Motion.Terminator.VariedTripwireTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;
@Autonomous
public class NewBlueAutonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    HashMap<String, Vector3> movements;
    int skystonePos;
    public NewBlueAutonomous() {
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
        movements.put("driveToClearSkystones", new Vector3(-10, 0, 0));
        movements.put("driveToFoundation", new Vector3(-14, -30, 0));
        movements.put("turnAndDriveToFoundation", new Vector3(-18, -40, -90));
        movements.put("moveFoundationToScoringZone", new Vector3(5, 0, 0));
        movements.put("driveBackToSkystones", new Vector3(-12.5, -15, 0));
        movements.put("driveToSecondSkystone1", new Vector3(-12.5, -15, 90));
        movements.put("driveToSecondSkystone2", new Vector3(-12.5, -20, 90));
        movements.put("driveToSecondSkystone3", new Vector3(-12.5, -25, 90));
        movements.put("driveToIntakeSecondSkystone", new Vector3(-15, 0, 90));
        final HashMap<String, Vector3> defaultTurns = new HashMap<>();
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
                            pos3 += byteMap[i];
                        }
                        pos3 = pos3 / 10;
                        for(int i = 10; i < 20; i ++){
                            pos2 += byteMap[i];
                        }
                        pos2 = pos2 / 10;
                        for(int i = 30; i < 36; i ++){
                            pos1 += byteMap[i];
                        }
                        pos1 = pos1 / 6;
                        int max = Math.min(Math.min(pos1, pos2), pos3);
                        telemetry.addData("Max1", pos3);
                        telemetry.addData("Max2", pos2);
                        telemetry.addData("Max3", pos1);
                        telemetry.addData("Position", max == pos1 ? "3" : (max == pos2 ? "2" : "1"));
                        skystonePos = (max == pos1 ? 3 : (max == pos2 ? 2 : 1));
                        telemetry.addData("SkystonePos", skystonePos);
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
                tripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), 5, 5);
                terminator = new CombinedORTerminator(position, Vector3.ZERO(), orientationTerminator, tripwireTerminator);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(0);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager driveToClearSkystones = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(movements.get("driveToClearSkystones"), 0.5));
                logicStates.put("latchOn", new LogicState(statemachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 150;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                        }
                    }
                });
                terminator = new OrientationTerminator(position, movements.get("driveToClearSkystones"), 4, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
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
                driveState.put("main", system.driveToPointSlowdown(movements.get("turnAndDriveToFoundation"), 0.4));
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
        StateMachineManager waitForLockOn = StateMachineManager.timer(150, statemachine);
        StateMachineManager driveFoundationToScoreZone = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void onStart(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
            }

            @Override
            public void setup() {
                driveState.put("main", system.driveForward(movements.get("moveFoundationToScoringZone"), 1));
                terminator = new OrientationTerminator(position, position.add(movements.get("moveFoundationToScoringZone")), 3, 1);
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
        StateMachineManager driveBackToSkystones = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            SideLaserTerminator sideLaserTerminator;
            CombinedANDTerminator combinedANDTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveToPoint(movements.get("driveBackToSkystones"), 1));
                orientationTerminator = new OrientationTerminator(position, Vector3.ZERO(), 4, 1);
                sideLaserTerminator = new SideLaserTerminator(position, Vector3.ZERO(), SideLaserTerminator.SIDE.RIGHT, SideLaserTerminator.DISTANCE.CLOSE, 5);
                combinedANDTerminator = new CombinedANDTerminator(position, Vector3.ZERO(), orientationTerminator, sideLaserTerminator);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedANDTerminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager driveBackForSecondSkystone = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                switch(skystonePos){
                    case 3:
                        driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone3"), 1));
                        terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone3"), 3, 0);
                        break;
                    case 2:
                        driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone2"), 1));
                        terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone2"), 3, 0);
                        break;
                    default:
                        driveState.put("main", system.driveToPoint(movements.get("driveToSecondSkystone1"), 1));
                        terminator = new OrientationTerminator(position, movements.get("driveToSecondSkystone1"), 3, 0);
                }
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(1);
                hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
            }
        };
        StateMachineManager moveToIntakeSecondSkystone = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            VariedTripwireTerminator variedTripwireTerminator;
            CombinedORTerminator combinedORTerminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveForward(movements.get("driveToIntakeSecondSkystone"), 1));
                orientationTerminator = new OrientationTerminator(position, position.add(movements.get("driveToIntakeSecondSkystone")), 2, 1);
                variedTripwireTerminator = new VariedTripwireTerminator(position, Vector3.ZERO(), SideLaserTerminator.DISTANCE.CLOSE, 3);
                combinedORTerminator = new CombinedORTerminator(position, Vector3.ZERO(), orientationTerminator, variedTripwireTerminator);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = combinedORTerminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakePowers(0);
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
            }
        };
        StateMachineManager clearBlocksV2 = new StateMachineManager(statemachine) {
            OrientationTerminator terminator;
            @Override
            public void setup() {
                driveState.put("main", system.driveForward(movements.get("driveToClearSkystones"), 0.5));
                terminator = new OrientationTerminator(position, position.add(movements.get("driveToClearSkystones")), 2, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = terminator.shouldTerminate(sensors);
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
            }
        };
        StateMachineManager driveToFoundationSecondTime = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

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
        stateMachineSwitcher.init(init, moveToSkystones, driveToIntakeBlock, driveToClearSkystones, driveToFoundation, turnToLatchOn, end);
    }
}
