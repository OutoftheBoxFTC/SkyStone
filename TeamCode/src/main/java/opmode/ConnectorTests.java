package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.HashMap;

import Debug.Connector;
import Debug.RecievePacket;
import Debug.Registers;
import Hardware.HardwareData;
import Hardware.SensorData;
import State.LogicState;
import State.StateMachineManager;

public class ConnectorTests extends BasicOpmode {
    Registers registers;
    public ConnectorTests(){
        super(0, true);
    }
    @Override
    public void setup() {
        HashMap<String, String> drive = new HashMap<>();
        drive.put("Test1", "4, -3, 0");
        drive.put("Test2", "-30, 15, 0");
        HashMap<String, String> turn = new HashMap<>();
        turn.put("Test3", "0, 0, 135");
        registers = new Registers(drive, turn);
        Connector.getInstance().addPathCoordinates(drive);
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("mainLogic", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        Connector.getInstance().addTelemetry("test", System.currentTimeMillis());
                        RecievePacket packet = Connector.getInstance().getDataFromMaster(500);
                        if(!(packet == null)) {
                            if (packet.positions.size() > 0) {
                                telemetry.addData("Tester", packet.positions.get(0));
                            }
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = gamepad1.b;
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
                        telemetry.addLine("Press X to start Autonomous");
                        terminate = gamepad1.x;
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
