package opmode.Testing.Sensors.Pixy;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.LogicState;
import State.StateMachineManager;
import opmode.BasicOpmode;

@TeleOp
public class PayloadServoTests extends BasicOpmode {
    public PayloadServoTests() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
                hardware.setCapstoneLatch(1);
            }
        };
        StateMachineManager main = new StateMachineManager(statemachine) {
            double servoPos = 0;
            int buttonState = 0;
            @Override
            public void setup() {
                logicStates.put("run", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.right_trigger > 0){
                            if(!stateMachine.logicStateActive("release")){
                                stateMachine.activateLogic("release");
                            }
                        }
                        telemetry.addData("servoPos", servoPos);
                    }
                });
                exemptedLogicstates.put("release", new LogicState(stateMachine) {
                    long timer = 0;
                    @Override
                    public void init(SensorData sensors, HardwareData hardware) {
                        timer = System.currentTimeMillis() + 1500;
                        hardware.setCapstoneLatch(0.2);
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(System.currentTimeMillis() > timer){
                            hardware.setCapstoneLatch(1);
                            deactivateThis();
                        }
                        telemetry.addData("E", "BEE");
                    }
                });
                exemptedLogicstates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad1.dpad_up && buttonState != 1){
                            servoPos += 0.1;
                            buttonState = 1;
                        }else if(gamepad1.dpad_down && buttonState != 2){
                            servoPos -= 0.1;
                            buttonState = 2;
                        }else if(gamepad1.y && buttonState != 3){
                            servoPos += 0.01;
                            buttonState = 3;
                        }else if(gamepad1.a && buttonState != 4){
                            servoPos -= 0.01;
                            buttonState = 4;
                        }else{
                            buttonState = 0;
                        }
                        if(gamepad1.right_trigger > 0.5){
                            hardware.setCapstoneLatch(HardwareConstants.CAPSTONE_LATCH_ON);
                        }else if(gamepad1.left_trigger > 0.5){
                            hardware.setCapstoneLatch(HardwareConstants.CAPSTONE_LATCH_OFF);
                        }else{
                            hardware.setCapstoneLatch(servoPos);
                        }
                        telemetry.addData("Servo Position", servoPos);
                        telemetry.addData("Actual Position", hardware.getCapstoneLatch());
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.init(init, main);
    }
}
