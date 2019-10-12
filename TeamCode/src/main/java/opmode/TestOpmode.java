package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import hardware.Hardware;
import hardware.ReadData;
import odometry.EvenDumberOdometry;
import state.LogicState;

@TeleOp(name = "Test")
public class TestOpmode extends BasicOpmode {
    EvenDumberOdometry odometer;
    public TestOpmode() {
        super(0, false);
    }

    @Override
    protected void setup() {
        //telemetry.enableLogger();
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.enableDevice(Hardware.HardwareDevice.GYRO);
        robot.enableDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        odometer = new EvenDumberOdometry(0.0010579445);
        HashMap<String, LogicState> stateList = new HashMap<>();
        stateList.put("orientation", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("Position", odometer.getPosition(data));
            }
        });
        stateList.put("-1", new LogicState(stateMachine) {

            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    stateMachine.activateLogic("orientation");
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicStates(stateList);
        stateMachine.activateLogic("-1");
    }
}