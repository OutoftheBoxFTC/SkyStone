package opmode.Testing.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import HardwareSystems.Hardware;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.StateMachineManager;
import opmode.BasicOpmode;

@TeleOp
public class SensorReadoutRightPixy extends BasicOpmode {
    public SensorReadoutRightPixy() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.enableDevice(Hardware.HardwareDevices.RIGHT_PIXY);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addLine("Press Start to Start. I hope that should have been obvious");
                terminate = isStarted();
            }
        };
        StateMachineManager main = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addData("Left Encoder", sensors.getLeft());
                telemetry.addData("Right Encoder", sensors.getRight());
                telemetry.addData("Aux Encoder", sensors.getAux());
                telemetry.addData("Gyro", sensors.getGyro());
                telemetry.addData("Lift", sensors.getLift());
                telemetry.addData("Right Pixy", sensors.getPixy()[sensors.getPixy().length-2] & 0xFF);
                telemetry.addData("Skystone?", (sensors.getPixy()[sensors.getPixy().length-2] & 0xFF) < 50);
                telemetry.addData("Intake Tripwire", sensors.getIntakeTripwire());
                telemetry.addData("Timestamp", sensors.getTimestamp());
                telemetry.addData("Time between frames", System.currentTimeMillis() - sensors.getTimestamp());
            }
        };
        stateMachineSwitcher.init(init, main);
    }
}