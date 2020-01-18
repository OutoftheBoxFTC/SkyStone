package opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import State.StateMachineManager;
@Autonomous
public class AbsoluteFinalWorstCaseEverythingIsGoingHorriblyWrong extends BasicOpmode {
    public AbsoluteFinalWorstCaseEverythingIsGoingHorriblyWrong() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                telemetry.addLine("Everything is going HORRIBLY wrong isn't it. I dislike this program");
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
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
        stateMachineSwitcher.start(init, run);
    }
}
