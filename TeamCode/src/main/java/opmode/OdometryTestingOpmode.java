package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Odometer.SimpleOdometer;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import state.LogicState;
@TeleOp
public class OdometryTestingOpmode extends BasicOpmode {
    public OdometryTestingOpmode() {
        super(0, false);
    }

    @Override
    protected void setup() {
        final SimpleOdometer odometer;
        final Vector3 position, velocity;
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.enableDevice(Hardware.HardwareDevice.GYRO);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(0.00105141415, position, velocity, telemetry);
        HashMap<String, LogicState> states = new HashMap<>();
        states.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(opModeIsActive()){
                    stateMachine.activateLogic("Tracking");
                    odometer.start(data);
                    deactivateThis();
                }
            }
        });
        states.put("Tracking", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                odometer.update(data);
                telemetry.setHeader("Position", position);
                telemetry.setHeader("Velocity", velocity);
                telemetry.setHeader("raw", new Vector3(data.getLeft(), data.getRight(), data.getAux()));
            }
        });
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("init");
    }
}
