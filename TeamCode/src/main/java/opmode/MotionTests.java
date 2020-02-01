package opmode;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.LogicState;
import State.StateMachineManager;
import math.Vector2;

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
            boolean test, test2;
            double tram = 0, trim = 0;
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
                        if(gamepad2.dpad_right && !test2){
                            test2 = true;
                            trim += 0.01;
                        }
                        if(!(gamepad2.dpad_right || gamepad2.dpad_left)){
                            test2 = false;
                        }
                        if(gamepad2.dpad_left && !test2){
                            test2 = true;
                            trim -= 0.01;
                        }
                        telemetry.addData("TRIMTRAM", (trim + " " + tram));
                        if(gamepad2.right_stick_x > 0.4){
                            hardware.setLiftServo(new Vector2(trim, tram));
                        }
                        if(gamepad2.right_stick_x < -0.4){
                            hardware.setLiftServo(new Vector2(trim, tram));
                        }
                        if(gamepad2.right_stick_y < -0.4){
                            hardware.setLiftServo(new Vector2(trim, tram));
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.init(init, move);
    }
}
