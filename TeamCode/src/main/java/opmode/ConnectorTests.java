package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

import Debug.Connector;
import Hardware.HardwareData;
import Hardware.SensorData;
import State.LogicState;
import State.StateMachineManager;

@TeleOp
public class ConnectorTests extends BasicOpmode {
    public ConnectorTests(){
        super(0);
    }
    @Override
    public void setup() {
        final Connector connector = Connector.getInstance();
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("mainLogic", new LogicState(stateMachine) {
                    @Override
                    public void init(SensorData sensors, HardwareData hardware){
                        try {
                            connector.start();
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                        connector.addTelemetry("test", "test");
                    }
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        try {
                            connector.update();
                            telemetry.addData("data", connector.getDataFromMaster(1000));
                            terminate = true;
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.start(main);
    }
}
