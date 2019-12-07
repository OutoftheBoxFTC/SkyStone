package opmode;

import Hardware.Hardware;
import Hardware.HardwareConstants;
import Hardware.HardwareData;
import Hardware.SensorData;
import Motion.MecanumSystem;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
        final StateMachineManager teleOpMode1 = new StateMachineManager(statemachine) {
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
                logicStates.put("lift", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(Math.abs(gamepad1.left_stick_y) < 0.1){
                            hardware.setLiftMotors(0.2);
                        }else{
                            if(sensors.getLift() > 0 && gamepad1.left_stick_y > 0) {
                                hardware.setLiftMotors(gamepad1.left_stick_y);
                            }else if(sensors.getLift() < 2150 && gamepad1.left_stick_y < 0){
                                hardware.setLiftMotors(gamepad1.left_stick_y);
                            }
                            else{
                                hardware.setLiftMotors(0.2);
                            }
                        }
                        if(gamepad2.x){
                            statemachine.activateLogic("raise");
                        }
                        if(gamepad2.b){
                            hardware.setIntakeLatch(0.3678);
                        }
                        if(gamepad2.right_trigger > 0){
                            hardware.setLiftServo(1);
                        }
                        if(gamepad2.left_trigger > 0){
                            hardware.setLiftServo(0);
                        }
                        if(gamepad1.left_bumper){
                            statemachine.activateLogic("outtakeSequence");
                        }
                        if(gamepad1.right_bumper){
                            statemachine.activateLogic("sequence");
                            statemachine.deactivateLogic("servo");
                        }

                        telemetry.addData("Encoder", sensors.getLift());

                    }
                });
                logicStates.put("servo", new LogicState(statemachine) {
                    double position = 0;
                    long timePrev = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.right_bumper){
                            position += 0.1 * ((System.currentTimeMillis() - timePrev)/1000.0);
                        }else if(gamepad2.left_bumper){
                            position -= 0.1 * ((System.currentTimeMillis() - timePrev)/1000.0);
                        }
                        position = Math.min(position, 1);
                        position = Math.max(position, 0);
                        hardware.setLiftServo(position);

                        telemetry.addData("Position", position);
                        timePrev = System.currentTimeMillis();
                    }
                });
                exemptedLogicstates.put("raise", new LogicState(statemachine) {
                    long timer = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(timer == 0) {
                            hardware.setLiftServo(0.5);
                            timer = System.currentTimeMillis() + 250;
                        }
                        if(timer < System.currentTimeMillis() && timer != 0){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                            timer = 0;
                        }
                    }
                });
                exemptedLogicstates.put("sequence", new LogicState(statemachine) {
                    long timer = 0;
                    int state = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(state == 0){
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                            timer = System.currentTimeMillis() + 250;
                            state = 1;
                        }
                        if(state == 1 && System.currentTimeMillis() > timer){
                            hardware.setLiftServo(0);
                            timer = System.currentTimeMillis() + 250;
                            state = 2;
                        }
                        if(state == 2 && System.currentTimeMillis() > timer){
                            hardware.setIntakeLatch(0.85);
                            timer = System.currentTimeMillis() + 250;
                            state = 3;
                        }
                        if(state == 3 && System.currentTimeMillis() > timer){
                            hardware.setLiftServo(0.02);
                            deactivateThis();
                            statemachine.activateLogic("servo");
                        }
                    }
                });
                exemptedLogicstates.put("outtakeSequence", new LogicState(statemachine) {
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
                            terminate = true;
                        }
                    }
                });
                logicStates.put("latchSystem", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.left_trigger > 0){
                            hardware.setLatchServos(HardwareConstants.LATCH_OFF);
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
                logicStates.put("Odometer", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Factor", sensors.getAux() / Math.toDegrees(sensors.getGyro()));                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = false;
            }
        };
        stateMachineSwitcher.start(initManager, teleOpMode1);
    }
}
