package opmode.Testing.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Odometer.SimpleOdometer;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
import opmode.BasicOpmode;

@TeleOp
public class SensorReadoutLeftPixy extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity;
    public SensorReadoutLeftPixy() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        robot.disableDevice(Hardware.HardwareDevices.SIDE_LASERS);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        HashMap<String, LogicState> nonManagedLogicStates = new HashMap<>();
        nonManagedLogicStates.put("Odometry", new LogicState(statemachine) {
            @Override
            public void init(SensorData sensors, HardwareData hardware){
                odometer.start(sensors);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                odometer.update(sensors);
                telemetry.addData("Position", position);
            }
        });
        statemachine.appendLogicStates(nonManagedLogicStates);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addLine("Press Start to Start. I hope that should have been obvious");
                terminate = isStarted();
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                odometer.start(sensors);
                statemachine.activateLogic("Odometry");
            }
        };
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addData("Position", position);
                telemetry.addData("Left Encoder", sensors.getLeft());
                telemetry.addData("Right Encoder", sensors.getRight());
                telemetry.addData("Aux Encoder", sensors.getAux());
                telemetry.addData("Gyro", sensors.getGyro());
                telemetry.addData("Lift", sensors.getLift());
                telemetry.addData("Left Pixy", sensors.getPixy()[sensors.getPixy().length-2] & 0xFF);
                telemetry.addData("Skystone?", (sensors.getPixy()[sensors.getPixy().length-2] & 0xFF) < 50);
                telemetry.addData("Intake Tripwire", sensors.getIntakeTripwire());
                telemetry.addData("Timestamp", sensors.getTimestamp());
                telemetry.addData("Time between frames", System.currentTimeMillis() - sensors.getTimestamp());
            }
        };
        stateMachineSwitcher.init(init, main);
    }
}
