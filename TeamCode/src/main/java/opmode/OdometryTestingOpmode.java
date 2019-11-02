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
    double adjustment = 0;
    @Override
    protected void setup() {
        final SimpleOdometer odometer;
        final Vector3 position, velocity;
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.enableDevice(Hardware.HardwareDevice.HUB_2_BULK);
        robot.enableDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.enableDevice(Hardware.HardwareDevice.GYRO);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        HashMap<String, LogicState> states = new HashMap<>();
        states.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(opModeIsActive()){
                    stateMachine.activateLogic("Tracking");
                    adjustment = data.getAux();
                    deactivateThis();
                }
            }
        });
        states.put("Tracking", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("AuxRaw", data.getAux() - adjustment);
                telemetry.setHeader("AuxFactor", ((data.getAux()-adjustment) / Math.PI));
                telemetry.setHeader("AuxDisplacement", 0.0423583858-((data.getAux()-adjustment) / Math.PI));
            }
        });
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("init");
    }
}
