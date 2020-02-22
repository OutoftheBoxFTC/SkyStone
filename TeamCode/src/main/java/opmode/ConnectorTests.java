package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.HashMap;

import Debug.Connector;
import Debug.RecievePacket;
import Debug.Registers;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.LogicState;
import State.StateMachineManager;
@TeleOp
public class ConnectorTests extends BasicOpmode {
    Registers registers;
    ArrayList<Double> graphVals = new ArrayList();
    int index = 0;
    public ConnectorTests(){
        super(0, true);
    }
    @Override
    public void setup() {
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("mainLogic", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        Connector.getInstance().addTelemetry("test", System.currentTimeMillis());
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                //terminate = gamepad1.b;
            }
        };
        StateMachineManager updateData = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("WaitForInput", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addLine("Press A to recieve data");
                        if(gamepad1.a){
                            statemachine.activateLogic("RecieveData");
                            deactivateThis();
                        }
                    }
                });
                exemptedLogicstates.put("RecieveData", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        RecievePacket packet = Connector.getInstance().getDataFromMaster(2000);
                        if(packet != null){
                            ArrayList<String> positions = new ArrayList<>();
                            ArrayList<String> turns = new ArrayList<>();
                            turns.addAll(packet.turns);
                            positions.addAll(packet.positions);
                            registers.setPoints(positions);
                            registers.setTurns(turns);
                        }
                        statemachine.activateLogic("WaitToStart");
                        deactivateThis();
                    }
                });
                exemptedLogicstates.put("WaitToStart", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        telemetry.addLine("Press X to init Autonomous");
                        terminate = gamepad1.x;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.init(main);
    }
}
