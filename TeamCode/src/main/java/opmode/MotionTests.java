package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Debug.Connector;
import Debug.Registers;
import Hardware.HardwareData;
import Hardware.SensorData;
import Motion.MotionSystem;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import math.Vector4;
@TeleOp
public class MotionTests extends BasicOpmode {
    Vector3 position = Vector3.ZERO();
    Vector3 velocity = Vector3.ZERO();
    public MotionTests() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        HashMap<String, String> test = new HashMap<>();
        test.put("Main", "0, -1, 0");
        final Registers registers = new Registers(test, new HashMap<String, String>());
        final SimpleOdometer odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem motion = new MotionSystem(statemachine, odometer, position);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("start", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        odometer.start(sensors);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }
        };
        StateMachineManager move = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("update", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        odometer.update(sensors);
                        telemetry.addData("Position", position.toString());
                        Connector.getInstance().addOrientation(position);
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.start(init, move);
    }
}
