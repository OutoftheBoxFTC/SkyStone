package opmode.Testing.Lift;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.LogicState;
import State.StateMachineManager;
import opmode.BasicOpmode;

@TeleOp
public class LiftSystem extends BasicOpmode {
    public LiftSystem() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.disableDevice(Hardware.HardwareDevices.SIDE_LASERS);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }
        };
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                //exemptedLogicstates.put("reset", HardwareConstants.resetLift(statemachine, "main"));
                logicStates.put("main", new LogicState(statemachine) {
                    double power = 0;
                    boolean updated = false;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad1.dpad_up || gamepad1.dpad_down) {
                            if(!updated){
                                power += gamepad1.dpad_up ? 0.025 : -0.025;
                                updated = true;
                            }
                        }else{
                            updated = false;
                        }
                        hardware.setLiftMotors(power);
                        if(gamepad1.y){
                            //statemachine.activateLogic("reset");
                            //deactivateThis();
                        }
                        telemetry.addData("Lift", sensors.getLift());
                        telemetry.addData("Reset", sensors.getLiftLimit());
                        telemetry.addData("Powers", power);
                        if(sensors.getLiftLimit()){
                            sensors.getCalibration().setLift(sensors.getRawLift());
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        StateMachineManager main2 = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        //hardware.setLiftMotors(gamepad1.dpad_up ? 1 : (gamepad1.dpad_down ? (sensors.getLiftLimit() ? 0 : -0.4) : 0.25));
                        hardware.setLiftMotors(gamepad1.left_stick_y);
                        if(gamepad1.y){
                            //statemachine.activateLogic("reset");
                            //deactivateThis();
                        }
                        telemetry.addData("Lift", sensors.getLift());
                        telemetry.addData("Reset", sensors.getLiftLimit());
                        telemetry.addData("Powers", gamepad1.dpad_up ? 1 : (gamepad1.dpad_down ? (sensors.getLiftLimit() ? 0 : -0.4) : 0.05));
                        if(sensors.getLiftLimit()){
                            sensors.getCalibration().setLift(sensors.getRawLift());
                        }
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
