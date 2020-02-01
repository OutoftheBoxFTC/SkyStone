package opmode;

import java.util.HashMap;

import Debug.Registers;
import HardwareSystems.*;
import Motion.MotionSystem;
import Motion.Terminator.OrientationTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;
public class NewRedAutonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    Registers registers;
    int skystonePos;
    public NewRedAutonomous() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        firstSkystone = Vector3.ZERO();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("moveToSkystone1", "-10, 0, 0");
        defaults.put("moveToSkystone2", "-10, 5, 0");
        defaults.put("moveToSkystone3", "-10, 10, 0");
        defaults.put("moveToFoundation", "-10, 24, 90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("rotateToGrabSkystone", "0, 0, 80");
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
                        for(int i = 0; i < 16; i ++){
                            pos3 += byteMap[i];
                        }
                        pos3 = pos3 / 16;
                        for(int i = 16; i < 30; i ++){
                            pos2 += byteMap[i];
                        }
                        pos2 = pos2 / 14;
                        for(int i = 30; i < 42; i ++){
                            pos1 += byteMap[i];
                        }
                        pos1 = pos1 / 12;
                        int max = Math.min(Math.min(pos1, pos2), pos3);
                        telemetry.addData("Max1", pos3);
                        telemetry.addData("Max2", pos2);
                        telemetry.addData("Max3", pos1);
                        telemetry.addData("Position", max == pos1 ? "1" : (max == pos2 ? "2" : "3"));
                        skystonePos = max == pos1 ? 1 : (max == pos2 ? 2 : 3);
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
                switch (skystonePos) {
                    case 2:
                        driveState.put("main", system.driveToPoint(registers.getPoint("moveToSkystone2"), 1));
                        break;
                    case 3:
                        driveState.put("main", system.driveToPoint(registers.getPoint("moveToSkystone3"), 1));
                        break;
                    default:
                        driveState.put("main", system.driveToPoint(registers.getPoint("moveToSkystone1"), 1));
                }
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                switch (skystonePos) {
                    case 2:
                        terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("moveToSkystone2"), 2);
                        break;
                    case 3:
                        terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("moveToSkystone3"), 2);
                        break;
                    default:
                        terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("moveToSkystone1"), 2);
                }
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
        stateMachineSwitcher.init(init, moveToSkystones, end);
    }
}
