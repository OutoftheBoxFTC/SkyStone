package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.HardwareConstants;
import Hardware.HardwareData;
import Hardware.SensorData;
import State.LogicState;
import State.StateMachineManager;
@TeleOp
public class LiftSystemsTests extends BasicOpmode {
    public LiftSystemsTests() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addLine("Press Start to start");
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        hardware.setLiftServo(0.02);
                        hardware.setIntakeLatch(0);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }
        };
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("lift", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setLiftMotors(gamepad1.left_stick_y);
                        if(gamepad1.a){
                            statemachine.activateLogic("sequence");
                            statemachine.deactivateLogic("servo");
                        }
                        if(gamepad1.x){
                            statemachine.activateLogic("raise");
                        }
                        if(gamepad1.b){
                            hardware.setIntakeLatch(0.3678);
                        }
                        telemetry.addData("Position", sensors.getLift());
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
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.start(init, main);
    }
}
