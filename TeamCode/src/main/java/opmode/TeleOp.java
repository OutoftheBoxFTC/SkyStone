package opmode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
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
        robot.disableDevice(Hardware.HardwareDevices.SIDE_LASERS);
        StateMachineManager initManager = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("init", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("In Init", true);
                        hardware.setLiftServo(HardwareConstants.LIFT_REST);
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
                exemptedLogicstates.put("reset", HardwareConstants.resetLift(statemachine, "lift"));
                driveState.put("drive", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        if(!gamepad1.left_bumper) {
                            return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x));
                        }else{
                            return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x/2, gamepad1.left_stick_y/2, -gamepad1.right_stick_x/2));
                        }
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
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
                        if(gamepad2.right_trigger > 0.5){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        }
                        if(((gamepad1.right_trigger > 0 || gamepad1.right_bumper || gamepad1.left_trigger > 0))){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                            hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                        }else if(hardware.getLiftServo().getA() == HardwareConstants.LIFT_INTAKE.getA()){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        }
                        if(gamepad2.dpad_left){
                            hardware.setLatchServos(HardwareConstants.LATCH_ON);
                        }
                        if(gamepad2.dpad_right){
                            hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                        }
                        if(gamepad2.right_stick_x > 0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        }
                        if(gamepad2.right_stick_x < -0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT);
                        }
                        if(gamepad2.right_stick_y < -0.4){
                            //hardware.setLiftServo(HardwareConstants.LIFT_MID);
                        }
                        if(gamepad2.right_stick_y > 0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
                        }
                        if(gamepad2.y){
                            stateMachine.activateLogic("reset");
                            deactivateThis();
                        }
                        if(sensors.getLiftLimit()){
                            sensors.getCalibration().setLift(sensors.getRawLift());
                        }
                        telemetry.addData("Encoder", sensors.getLift());

                    }
                });
                logicStates.put("intake", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad1.right_trigger > 0){
                            hardware.setIntakePowers(gamepad1.right_trigger);
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE_TELEOP);
                        }else if(gamepad1.right_bumper){
                            hardware.setIntakePowers(1);
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        }else if(gamepad1.left_trigger > 0){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE_TELEOP);
                            hardware.setIntakePowers(-gamepad1.left_trigger);
                        }else{
                            hardware.setIntakePowers(0);
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        }
                        if(sensors.getIntakeTripwire() < 6){
                            hardware.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
                        }else{
                            hardware.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                        }
                        telemetry.addData("Tripwire", sensors.getIntakeTripwire());
                    }
                });
                logicStates.put("closeLatch", new LogicState(statemachine) {
                    long timer = 0;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){

                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if((gamepad1.right_trigger > 0 || gamepad1.right_bumper || gamepad1.left_trigger > 0)){
                            timer = System.currentTimeMillis() + 150;
                        }
                        if(gamepad2.right_trigger > 0){
                            timer = System.currentTimeMillis() + 20000;
                        }
                        if(System.currentTimeMillis() > timer){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_ON);
                        }
                    }
                });
                logicStates.put("capstone", new LogicState(statemachine) {
                    public boolean prevState = false, servoState = false;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.y != prevState && gamepad2.y){
                            servoState = !servoState;
                        }
                        prevState = gamepad2.y;
                        hardware.setCapstoneLatch(servoState ? HardwareConstants.CAPSTONE_LATCH_ON : HardwareConstants.CAPSTONE_LATCH_OFF);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = false;
            }
        };
        stateMachineSwitcher.init(initManager, teleOpMode1);
    }
}
