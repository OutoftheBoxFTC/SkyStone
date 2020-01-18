package opmode;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.LogicState;
import State.StateMachineManager;

public class PixyTest extends BasicOpmode{
    public PixyTest() {
        super(0, true);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        int test = ((sensors.getPixy()[sensors.getPixy().length-2] & 0xFF));
                        telemetry.addData("SkyStone?", Math.abs(test) < 140);
                        for(int i = 0; i < sensors.getPixy().length; i ++){
                            telemetry.addData("Data " + i, sensors.getPixy()[i] & 0xFF);
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = false;
            }
        };
        stateMachineSwitcher.start(main);
    }
}
