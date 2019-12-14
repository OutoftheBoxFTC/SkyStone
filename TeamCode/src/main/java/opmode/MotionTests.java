package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.HardwareConstants;
import Hardware.HardwareData;
import Hardware.SensorData;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
@TeleOp(name="TrimTram")
public class MotionTests extends BasicOpmode {
    public MotionTests() {
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
            }
        };
        StateMachineManager move = new StateMachineManager(statemachine) {
            boolean test;
            double tram = 0;
            @Override
            public void setup() {
                logicStates.put("update", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad2.dpad_up && !test){
                            test = true;
                            tram += 0.01;
                        }
                        if(!(gamepad2.dpad_up || gamepad2.dpad_down)){
                            test = false;
                        }
                        if(gamepad2.dpad_down && !test){
                            test = true;
                            tram -= 0.01;
                        }
                        telemetry.addData("TRIMTRAM", tram);
                        if(gamepad2.right_stick_x > 0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_OUT, tram);
                        }
                        if(gamepad2.right_stick_x < -0.4){
                            hardware.setLiftServo(HardwareConstants.LIFT_REST, tram);
                        }
                        if(gamepad2.right_stick_y < -0.4){
                            hardware.setLiftServo(0.5, tram);
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.start(init, move);
    }
}
