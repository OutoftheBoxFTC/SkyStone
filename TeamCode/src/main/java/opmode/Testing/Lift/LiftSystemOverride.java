package opmode.Testing.Lift;

import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.LogicState;
import State.StateMachineManager;
import opmode.BasicOpmode;

public class LiftSystemOverride extends BasicOpmode {
    public LiftSystemOverride() {
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
                        telemetry.addLine("WARNING! This program has NO encoder stops. BE VERY CAREFUL RUNNING. Press START to continue");
                        hardware.setIntakeServos(HardwareConstants.CLOSE_INTAKE);
                        hardware.setLiftServo(HardwareConstants.LIFT_REST);
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
                        hardware.setLiftMotors(-gamepad2.left_stick_y);
                        telemetry.addLine("WARNING! This program has NO encoder stops. BE VERY CAREFUL RUNNING");
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
