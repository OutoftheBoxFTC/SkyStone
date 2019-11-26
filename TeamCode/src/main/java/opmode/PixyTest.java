package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Debug.Connector;
import Hardware.HardwareData;
import Hardware.SensorData;
import State.LogicState;
import State.StateMachineManager;

@TeleOp
public class PixyTest extends BasicOpmode{
    public PixyTest() {
        super(0, true);
    }

    @Override
    public void setup() {
        robot.enableAll();
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        int test = sensors.getPixy()[sensors.getPixy().length-2] & 0xFF;
                        telemetry.addData("SkyStone?", test < 120);
                        for(int i = 0; i < sensors.getPixy().length; i ++){
                            telemetry.addData("Data " + i, sensors.getPixy()[i]);
                            Connector.getInstance().addTelemetry("Data " + i, sensors.getPixy()[i]);
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
