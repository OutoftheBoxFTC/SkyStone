package opmode;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.HashMap;

import Hardware.HardwareData;
import Hardware.SensorData;
import Motion.MotionSystem;
import Odometer.SimpleOdometer;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;

public class BlueAutonomous extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity;
    public BlueAutonomous() {
        super(1);
    }

    @Override
    public void setup() {
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem system = new MotionSystem(statemachine, odometer, position);
        HashMap<String, LogicState> permanenetLogicStates = new HashMap<>();
        permanenetLogicStates.put("Odometry", new LogicState(statemachine) {
            @Override
            public void init(SensorData sensors, HardwareData hardware){
                odometer.start(sensors);
            }
            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                odometer.update(sensors);
            }
        });
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }
        };
        StateMachineManager driveToFoundation = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                logicStates.put("drive", system.driveToPoint(new Vector3(9, -17, 0), 0.3));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
    }
}
