package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import hardware.Hardware;
import hardware.ReadData;
import state.LogicState;
@TeleOp
public class TranslationFactorTest extends BasicOpmode {
    public TranslationFactorTest(){
        super(0, false);
    }

    @Override
    public void setup(){
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        HashMap<String, LogicState> states = new HashMap<>();
        states.put("test", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("position", (data.getLeft() + data.getRight())/2);
            }
        });
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("test");
    }
}
