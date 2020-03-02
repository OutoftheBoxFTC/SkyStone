package opmode.Autonomous.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.Terminator.TimerTerminator;
import State.StateMachineManager;
import State.VelocityDriveState;
import math.Vector3;
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
            TimerTerminator terminator;
            @Override
            public void setup() {
                terminator = new TimerTerminator(Vector3.ZERO(), Vector3.ZERO(), 500);
                driveState.put("main", new VelocityDriveState(stateMachine) {
                    @Override
                    public Vector3 getVelocities() {
                        return new Vector3(0, 0.4, 0);
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                hardware.setLatchServos(HardwareConstants.LATCH_ON);
                terminate = terminator.shouldTerminate(sensors);
            }
        };
        StateMachineManager end = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("main", new VelocityDriveState(stateMachine) {
                    @Override
                    public Vector3 getVelocities() {
                        return Vector3.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = false;
            }
        };
        stateMachineSwitcher.init(init, run, end);
    }
}
