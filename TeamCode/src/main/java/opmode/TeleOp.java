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
                        hardware.setLiftServo(HardwareConstants.LIFT_REST, HardwareConstants.LIFT_REST_OFFSET);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }
        };
        final StateMachineManager teleOpMode1 = new StateMachineManager(statemachine) {
            boolean slowdown = false, prevState = false;
            @Override
            public void setup() {
                driveState.put("drive", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        if(!slowdown) {
                            return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x));
                        }else{
                            return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x/2, gamepad1.left_stick_y/2, -gamepad1.right_stick_x/2));
                        }
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(prevState != gamepad1.a){
                            if(gamepad1.a){
                                slowdown = !slowdown;
                            }
                            prevState = gamepad1.a;
                        }
                    }
                });
                logicStates.put("lift", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(Math.abs(gamepad2.left_stick_y) < 0.1){
                            hardware.setLiftMotors(0.2);
                        }else if(gamepad2.right_bumper){
                            hardware.setLiftMotors(-gamepad2.left_stick_y);
                        } else{
                            if(sensors.getLift() > 0 && gamepad2.left_stick_y > 0) {
                                hardware.setLiftMotors(-gamepad2.left_stick_y * (gamepad2.left_trigger > 0.1 ? 0.5 : 1));
                            }else if(sensors.getLift() < 2150 && gamepad2.left_stick_y < 0){
                                hardware.setLiftMotors(-gamepad2.left_stick_y * (gamepad2.left_trigger > 0.1 ? 0.5 : 1));
                            }
                            else{
                                hardware.setLiftMotors(0.2);
                            }
                        }
                        if(gamepad2.left_bumper){
                            robot.calibrate(robot.getCalibration().setLift(sensors.getLift()));
                        }
                        if(gamepad1.right_trigger > 0 || gamepad1.right_bumper || gamepad1.left_trigger > 0 || gamepad2.right_trigger > 0.5){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                            hardware.setLiftServo(0.08);
                        }else if(hardware.getLiftServoDouble() != HardwareConstants.LIFT_OUT && hardware.getLiftServoDouble() != 0.5){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                        }
                        if(gamepad2.dpad_left){
                            hardware.setLatchServos(HardwareConstants.LATCH_ON);
                        }
                        if(gamepad2.dpad_right){
                            hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                        }
                        if(gamepad2.right_stick_x > 0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT, HardwareConstants.LIFT_OUT_OFFSET);
                        }
                        if(gamepad2.right_stick_x < -0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST, HardwareConstants.LIFT_REST_OFFSET);
                        }
                        if(gamepad2.right_stick_y < -0.4){
                            hardware.setLiftServo(0.5, (HardwareConstants.LIFT_REST_OFFSET + HardwareConstants.LIFT_OUT_OFFSET) / 2);
                        }
                        telemetry.addData("Encoder", sensors.getLift());

                    }
                });
                logicStates.put("intake", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad1.right_trigger > 0){
                            hardware.setIntakePowers(gamepad1.right_trigger);
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }else if(gamepad1.right_bumper){
                            hardware.setIntakePowers(1);
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        }else if(gamepad1.left_trigger > 0){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                            hardware.setIntakePowers(-gamepad1.left_trigger);
                        }else{
                            hardware.setIntakePowers(0);
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        }
                    }
                });
                logicStates.put("Odometer", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Factor", sensors.getAux() / Math.toDegrees(sensors.getGyro()));
                    }
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
