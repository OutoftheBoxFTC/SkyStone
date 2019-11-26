package opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.HashMap;

import Debug.Connector;
import Debug.RecievePacket;
import Debug.Registers;
import Hardware.HardwareConstants;
import Hardware.HardwareData;
import Hardware.SensorData;
import Motion.MotionSystem;
import Motion.Terminator.OrientationTerminator;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;

@Autonomous
public class BlueAutonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity;
    Registers registers;
    public BlueAutonomous() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("driveToFoundation", "4, -14, 0");
        defaults.put("driveBack", "4, -1, 0");
        defaults.put("driveToSkystones", "-20, -13, 90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("turnToSkystones", "9, -1, 90");
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
                terminate = isStarted();
                stateMachine.activateLogic("Odometry");
            }
        };
        StateMachineManager driveToFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToFoundation"), 0.35));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
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
                        timer = System.currentTimeMillis() + 1000;
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
        StateMachineManager driveFoundationBack = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveBack"), 0.5));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveBack"), 2);
            }
        };
        StateMachineManager turnToSkystones = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.turn(registers.getTurn("turnToSkystones")));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminateRotation(position.getC(), registers.getTurn("turnToSkystones").getC(), 5);
            }
        };
        StateMachineManager driveToSkystones = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("driveToSkystones"), 0.75));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("driveToSkystones"), 4);
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
                        /*RecievePacket packet = Connector.getInstance().getDataFromMaster(2000);
                        if(packet != null){
                            ArrayList<String> positions = new ArrayList<>();
                            ArrayList<String> turns = new ArrayList<>();
                            turns.addAll(packet.turns);
                            positions.addAll(packet.positions);
                            registers.setPoints(positions);
                            registers.setTurns(turns);
                        }
                        */statemachine.activateLogic("WaitToStart");
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
        stateMachineSwitcher.start(init, driveToFoundation, latchOnToFoundation, driveFoundationBack, end);
    }
}
