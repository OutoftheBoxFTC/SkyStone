package opmode;

import Hardware.HardwareConstants;
import Hardware.HardwareData;
import Hardware.SensorData;
import Motion.MecanumSystem;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;

public class TeleOp extends BasicOpmode {
    public TeleOp() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        StateMachineManager initManager = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("init", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("In Init", true);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }
        };
        StateMachineManager teleOpMode1 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x));
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("latchSystem", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.left_trigger > 0){
                            hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                        }
                        if(gamepad2.right_trigger > 0){
                            hardware.setLatchServos(HardwareConstants.DUMP_BLOCK);
                        }
                        if(gamepad2.x){
                            hardware.setLatchServos(HardwareConstants.LATCH_ON);
                        }
                    }
                });
                logicStates.put("intake", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.right_trigger > 0){
                            hardware.setIntakePowers(gamepad2.right_trigger);
                        }else{
                            hardware.setIntakePowers(-gamepad2.left_trigger);
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = gamepad1.x;
            }
        };
        StateMachineManager teleOpMode2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", new DriveState(stateMachine) {
                    double offset = 0;
                    @Override
                    public Vector4 getWheelVelocities(SensorData data) {
                        double r = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);
                        if(gamepad1.right_bumper){
                            offset = data.getGyro();
                        }
                        double theta = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (data.getGyro()-offset);
                        return MecanumSystem.translate(new Vector3(r * Math.cos(theta), r * Math.sin(theta), -gamepad1.right_stick_x));
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                logicStates.put("latchSystem", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.left_trigger > 0){
                            hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                        }
                        if(gamepad2.right_trigger > 0){
                            hardware.setLatchServos(HardwareConstants.DUMP_BLOCK);
                        }
                        if(gamepad2.x){
                            hardware.setLatchServos(HardwareConstants.LATCH_ON);
                        }
                    }
                });
                logicStates.put("intake", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.right_trigger > 0){
                            hardware.setIntakePowers(gamepad2.right_trigger, gamepad2.right_trigger * -0.5);
                        }else{
                            hardware.setIntakePowers(-gamepad2.left_trigger, -gamepad2.left_trigger * -0.5);
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = gamepad1.y;
            }
        };
        stateMachineSwitcher.start(initManager, teleOpMode1, teleOpMode2);
    }
}
