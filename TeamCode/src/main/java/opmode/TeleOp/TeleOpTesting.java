package opmode.TeleOp;

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
import opmode.BasicOpmode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpTesting extends BasicOpmode {
    int currPosition = 1;
    boolean capInit = false;
    MediaPlayer mediaPlayer;
    double[] positions = {1, 40, 160, 260, 420, 590, 715, 840, 1010, 1120, 1240, 1440, 1540};
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
                mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.boathorn);
                mediaPlayer.setLooping(true);
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
                            return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x * 0.4, gamepad1.left_stick_y * 0.4, -gamepad1.right_stick_x * 0.4));
                        }
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Current Position", currPosition);
                        telemetry.addData("Lift", sensors.getLift());
                        telemetry.addData("Limit", sensors.getLiftLimit());
                        if(sensors.getLiftLimit()){
                            sensors.getCalibration().setLift(sensors.getRawLift());
                        }
                        if(gamepad2.b){
                            if(!mediaPlayer.isPlaying()){
                                mediaPlayer.start();
                            }
                        }else{
                            if(mediaPlayer.isPlaying()){
                                mediaPlayer.pause();
                                mediaPlayer.seekTo(0);
                            }
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
                            //robot.calibrate(robot.getCalibration().setLift(sensors.getLift()));
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
                exemptedLogicstates.put("resetMain", new LogicState(stateMachine) {
                    long timer = 0;

                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 500;
                        hardware.setLiftServo(HardwareConstants.LIFT_REST);
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() >= timer){
                            stateMachine.activateLogic("reset");
                            deactivateThis();
                        }
                    }
                });
                exemptedLogicstates.put("reset", new LogicState(stateMachine) {
                    int state = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(state == 0) {
                            if (sensors.getLiftLimit()) {
                                hardware.setLiftMotors(0.7);
                            }else{
                                state = 1;
                            }
                        }else if(state == 1){
                            if (sensors.getLiftLimit()) {
                                state = 2;
                            }else{
                                hardware.setLiftMotors(-0.4);
                            }
                        }else if(state == 2){
                            hardware.setLiftMotors(0);
                            stateMachine.activateLogic("lift");
                            state = 0;
                            deactivateThis();
                        }
                        if(Math.abs(gamepad2.left_stick_y) > 0.1){
                            hardware.setLiftMotors(0);
                            stateMachine.activateLogic("lift");
                            state = 0;
                            deactivateThis();
                        }
                    }
                });
                logicStates.put("lift", new LogicState(statemachine) {
                    boolean changeActive = false;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(currPosition >= positions.length){
                            currPosition = positions.length-1;
                        }
                        telemetry.addData("gamepad2", gamepad2.left_stick_y);
                        if(gamepad2.dpad_up){
                            if(!changeActive) {
                                currPosition++;
                                if (currPosition > 12) {
                                    currPosition = 12;
                                }
                                changeActive = true;
                            }
                        }else if(gamepad2.dpad_down){
                            if(!changeActive) {
                                currPosition--;
                                if (currPosition < 1) {
                                    currPosition = 1;
                                }
                                changeActive = true;
                            }
                        }else{
                            changeActive = false;
                        }
                        if(gamepad2.right_stick_y > 0.95){
                            stateMachine.activateLogic("resetMain");
                            deactivateThis();
                        }else if(gamepad2.right_stick_y < -0.95){
                            stateMachine.activateLogic("raiseLift");
                            deactivateThis();
                        }
                        if(Math.abs(gamepad2.left_stick_y) > 0.5) {
                            if((-gamepad2.left_stick_y) < 0){
                                if(!sensors.getLiftLimit()) {
                                    if(gamepad2.left_trigger > 0.1) {
                                        if(sensors.getLift() > 700) {
                                            hardware.setLiftMotors((-gamepad2.left_stick_y * 0.4) + 0.4);
                                        }else if(sensors.getLift() > 300){
                                            hardware.setLiftMotors((-gamepad2.left_stick_y * 0.4) + 0.3);
                                        }else{
                                            hardware.setLiftMotors((-gamepad2.left_stick_y * 0.4) + 0.1);
                                        }
                                    }else{
                                        hardware.setLiftMotors(Math.max((sensors.getLift() / 10) * (-gamepad2.left_stick_y * 0.05), -0.3));
                                    }
                                }
                            }else{
                                hardware.setLiftMotors(-gamepad2.left_stick_y);
                            }
                        }else{
                            hardware.setLiftMotors(0.25);
                        }
                        if(gamepad2.right_trigger > 0.3){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                        }
                        if(((gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0))){
                            hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
                            hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
                        }else if(hardware.getLiftServo().getA() == HardwareConstants.LIFT_INTAKE.getA()){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        }
                    }
                });
                exemptedLogicstates.put("adjustLift", new LogicState(statemachine) {
                    int numFrames = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftMotors(sensors.getLift() < positions[currPosition] ? ((positions[currPosition] - sensors.getLift())/10) : -0.4);
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
                            hardware.setLiftMotors(Math.max(0.9, (positions[currPosition] - sensors.getLift())/50));
                            if ((sensors.getLift()-50) >= positions[currPosition]) {
                                state = 1;
                            }
                        }else if(state == 1){
                            hardware.setLiftMotors(0.9);
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
                        if(Math.abs(gamepad2.left_stick_y) > 0.1){
                            hardware.setLiftMotors(0);
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            stateMachine.activateLogic("lift");
                            state = 0;
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
                            hardware.setLiftMotors(-0.4);
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            if((sensors.getLiftLimit())){
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
                            if(gamepad1.right_bumper){
                                hardware.setIntakePowers(-1);
                            }else {
                                hardware.setIntakePowers(0);
                            }
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
                            hardware.setLiftServo(HardwareConstants.LIFT_SCORING_POSITION);
                        }
                        if(gamepad2.y){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT);
                        }
                        if(gamepad2.left_bumper){
                            hardware.setCapstoneLatch(0.2);
                            capInit = true;
                        }else{
                            if(capInit) {
                                hardware.setCapstoneLatch(1);
                            }
                        }
                        telemetry.addData("Tripwire", sensors.getIntakeTripwire());
                    }
                });
                logicStates.put("intakeServo", new LogicState(statemachine) {
                    int frames = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(Math.abs(hardware.getIntakePowers().getA()) > 0.1) {
                            if (sensors.getIntakeTripwire() <= 9) {
                                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                            }
                        }else{
                            if(gamepad1.right_bumper){
                                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                            }else {
                                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                            }
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
                exemptedLogicstates.put("capstone", new LogicState(statemachine) {
                    public boolean prevState = false, servoState = false;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.y != prevState && gamepad2.y){
                            servoState = !servoState;
                        }
                        prevState = gamepad2.y;
                        //hardware.setCapstoneLatch(servoState ? HardwareConstants.CAPSTONE_LATCH_ON : HardwareConstants.CAPSTONE_LATCH_OFF);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = gamepad1.dpad_up;
            }
        };
        StateMachineManager feedMode = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        if(!gamepad1.left_bumper) {
                            return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x));
                        }else{
                            return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x * 0.3, gamepad1.left_stick_y * 0.3, -gamepad1.right_stick_x * 0.3));
                        }
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addData("Current Position", currPosition);
                        telemetry.addData("Lift", sensors.getLift());
                        telemetry.addData("Limit", sensors.getLiftLimit());
                        if(sensors.getLiftLimit()){
                            sensors.getCalibration().setLift(sensors.getRawLift());
                        }
                        if(gamepad2.b){
                            if(!mediaPlayer.isPlaying()){
                                mediaPlayer.start();
                            }
                        }else{
                            if(mediaPlayer.isPlaying()){
                                mediaPlayer.pause();
                                mediaPlayer.seekTo(0);
                            }
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
                            if(gamepad1.right_bumper){
                                hardware.setIntakePowers(-1);
                            }else {
                                hardware.setIntakePowers(0);
                            }
                        }
                        if(((gamepad1.right_trigger > 0 || gamepad1.right_bumper || gamepad1.left_trigger > 0))){
                            hardware.setLiftServo(HardwareConstants.LIFT_INTAKE);
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
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_FEED);
                        if(gamepad1.left_trigger > 0){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST);
                            if(sensors.getLift() < 20){
                                hardware.setLiftMotors(0.7);
                            }else{
                                hardware.setLiftMotors(0);
                            }
                        }else {
                            if(!sensors.getLiftLimit()){
                                hardware.setLiftMotors(-0.4);
                            }else{
                                hardware.setLiftMotors(0);
                            }
                        }
                        if(sensors.getLiftLimit()){
                            sensors.getCalibration().setLift(sensors.getRawLift());
                        }
                        telemetry.addData("Tripwire", sensors.getIntakeTripwire());
                        telemetry.addLine("In Feed Mode");
                    }
                });
                logicStates.put("intakeServo", new LogicState(statemachine) {
                    int frames = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(Math.abs(hardware.getIntakePowers().getA()) > 0.1) {
                            if (sensors.getIntakeTripwire() <= 9) {
                                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                            }
                        }else{
                            if(gamepad1.right_bumper){
                                hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                            }else {
                                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                            }
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = gamepad1.dpad_down;
            }
        };
        stateMachineSwitcher.init(initManager, teleOpMode1, feedMode);
    }
}
