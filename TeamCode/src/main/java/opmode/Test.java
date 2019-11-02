package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import hardware.ReadData;
import state.LogicState;
@TeleOp
public class Test extends BasicOpmode {
    int i = 0;
    public Test() {
        super(0, true);
    }

    @Override
    protected void setup() {
        HashMap<String, LogicState> states = new HashMap<>();
        states.put("main", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("Hello", i);
                i ++;
            }
        });
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("main");
    }
}
