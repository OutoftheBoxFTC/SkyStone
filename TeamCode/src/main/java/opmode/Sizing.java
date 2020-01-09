package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.HardwareConstants;
import Hardware.HardwareData;
import Hardware.SensorData;
import State.LogicState;
import State.StateMachineManager;
import math.Vector2;
@TeleOp
public class Sizing extends BasicOpmode {
    public Sizing() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
                        hardware.setLatchServos(new Vector2(0.95, 0.1));
                        hardware.setLiftServo(HardwareConstants.LIFT_REST);
                        hardware.setIntakeLatch(HardwareConstants.INTAKE_LATCH_OFF);
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
