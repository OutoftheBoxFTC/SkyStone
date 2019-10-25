package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import hardware.Hardware;
import hardware.ReadData;
import state.LogicState;
@TeleOp
public class PixycamTests extends BasicOpmode {
    public PixycamTests(){
        super(0, false);
    }
    @Override
    protected void setup() {
        robot.enableDevice(Hardware.HardwareDevice.PIXYCAM);
        HashMap<String, LogicState> states = new HashMap<>();
        states.put("pixyData", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(ReadData data) {
                if(timer == 0 || System.currentTimeMillis() > timer) {
                    robot.getPixy().queueData();
                    timer = System.currentTimeMillis() + 25;
                }
                telemetry.setHeader("X", robot.getPixy().getY());
                for(int i = 0; i < robot.getPixy().getShorts().size(); i ++){
                    telemetry.setHeader(String.valueOf(i), robot.getPixy().getShorts().get(i));
                }
            }
        });
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("pixyData");
    }
}
