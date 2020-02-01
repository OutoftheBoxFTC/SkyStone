package opmode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.R;

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
public class TeleOpTesting extends BasicOpmode {
    MediaPlayer player;
    int currPosition = 0;
    double[] positions = {1, 80, 180, 280, 380, 480, 580, 680, 780, 880, 980, 1080, 1180};
    public TeleOpTesting() {
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

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                player = MediaPlayer.create(hardwareMap.appContext, R.raw.chimeconnect);
                player.setLooping(true);
            }
        };
        final StateMachineManager teleOpMode1 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
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
                        telemetry.addData("Playing", player.isPlaying());
                        if(gamepad1.dpad_up){
                            if(!player.isPlaying()) {
                                player.start();
                            }
                        }else{
                            if(player.isPlaying()){
                                player.pause();
                                player.seekTo(0);
                            }
                        }
                        telemetry.addData("Current Position", currPosition);
                        telemetry.addData("Lift", sensors.getLift());
                        telemetry.addData("Limit", sensors.getLiftLimit());
                        if(sensors.getLiftLimit()){
                            sensors.getCalibration().setLift(sensors.getRawLift());
                        }
                    }
                });
                exemptedLogicstates.put("liftOld", new LogicState(statemachine) {
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
                            //hardware.setLiftServo(HardwareConstants.LIFT_OUT);
                        }
                        telemetry.addData("Encoder", sensors.getLift());

                    }
                });
                logicStates.put("firstReset", HardwareConstants.resetLift(statemachine, "lift"));
                exemptedLogicstates.put("reset", HardwareConstants.resetLift(statemachine, "lift"));
                exemptedLogicstates.put("lift", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(currPosition >= positions.length){
                            currPosition = positions.length-1;
                        }
                        if(gamepad2.dpad_up){
                            statemachine.activateLogic("raiseLift");
                            deactivateThis();
                        }
                        if(gamepad2.y){
                            stateMachine.activateLogic("reset");
                            deactivateThis();
                        }
                        if(Math.abs(gamepad2.left_stick_y) > 0.1) {
                            if((-gamepad2.left_stick_y) < 0){
                                if(!sensors.getLiftLimit()) {
                                    hardware.setLiftMotors(-gamepad2.left_stick_y * 0.4);
                                }
                            }else{
                                hardware.setLiftMotors(-gamepad2.left_stick_y);
                            }
                        }else{
                            hardware.setLiftMotors(0.25);
                        }
                    }
                });
                exemptedLogicstates.put("adjustLift", new LogicState(statemachine) {
                    int numFrames = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftMotors(sensors.getLift() < positions[currPosition] ? 1 : -0.4);
                        if(Math.abs(sensors.getLift() - positions[currPosition]) < 50){
                            numFrames ++;
                            if(numFrames >= 15) {
                                numFrames = 0;
                                statemachine.activateLogic("lift");
                                deactivateThis();
                            }
                        }
                    }
                });
                exemptedLogicstates.put("raiseLift", new LogicState(statemachine) {
                    int state;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        state = 0;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(state == 0) {
                            hardware.setLiftMotors(Math.max(0.4, ((sensors.getLift()-50) / positions[currPosition])/10));
                            if ((sensors.getLift()-50) >= positions[currPosition]) {
                                state = 1;
                            }
                        }else if(state == 1){
                            hardware.setLiftMotors(0.4);
                            hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
                            if((sensors.getLift()) >= positions[currPosition]){
                                state = 2;
                                hardware.setLiftMotors(0.25);
                            }
                        }else if(state == 2){
                            hardware.setLiftMotors(0.25);
                            statemachine.activateLogic("lift");
                            currPosition ++;
                            deactivateThis();
                        }
                    }
                });
                exemptedLogicstates.put("lowerLift", new LogicState(statemachine) {
                    int state;
                    long timer = 0;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        state = 0;
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(state == 0) {
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            timer = (System.currentTimeMillis() + 251);
                            if(System.currentTimeMillis() >= timer){
                                state = 1;
                            }
                        }else if(state == 1){
                            hardware.setLiftMotors(-1);
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            if((sensors.getLift()) >= positions[currPosition]){
                                state = 2;
                                hardware.setLiftMotors(0);
                            }
                        }else if(state == 2){
                            hardware.setLiftMotors(0);
                            statemachine.activateLogic("lift");
                            deactivateThis();
                        }
                    }
                });
                logicStates.put("intake", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad1.right_trigger > 0){
                            hardware.setIntakePowers(gamepad1.right_trigger);
                        }else if(gamepad1.left_trigger > 0){
                            hardware.setIntakePowers(-gamepad1.left_trigger);
                        }else{
                            hardware.setIntakePowers(0);
                        }
                        if(sensors.getIntakeTripwire() <= 12){
                            hardware.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        }
                        if(sensors.getIntakeTripwire() <= 6){
                            hardware.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                        }
                        if(sensors.getIntakeTripwire() <= 0.75){
                            hardware.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        }
                        if(sensors.getIntakeTripwire() > 12){
                            hardware.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                        }
                        if(gamepad2.dpad_left){
                            hardware.setLatchServos(HardwareConstants.LATCH_ON);
                        }
                        if(gamepad2.dpad_right){
                            hardware.setLatchServos(HardwareConstants.LATCH_OFF);
                        }
                        if(gamepad2.right_stick_x < -0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        }
                        if(gamepad2.right_stick_x > 0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT);
                        }
                        telemetry.addData("Tripwire", sensors.getIntakeTripwire());
                    }
                });
                logicStates.put("intakeServo", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(sensors.getIntakeTripwire() <= 12){
                            hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        }else{
                            hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        }
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
