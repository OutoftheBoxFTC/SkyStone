package opmode.Autonomous.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.StateMachineManager;
import opmode.BasicOpmode;

@Autonomous
public class LatchDownPark extends BasicOpmode {
    public LatchDownPark() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                telemetry.addData("Really?", "Everything is going HORRIBLY wrong isn't it. I dislike this program");
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
                hardware.setLiftServo(HardwareConstants.LIFT_REST);
                hardware.setIntakeServos(HardwareConstants.OPEN_INTAKE);
            }
        };
        StateMachineManager run = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_ON);
            }
        };
        stateMachineSwitcher.init(init, run);
    }
}
