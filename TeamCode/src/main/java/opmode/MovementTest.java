package opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.HashMap;

import Debug.Registers;
import Hardware.HardwareData;
import Hardware.SensorData;
import Motion.MotionSystem;
import Motion.Terminator.OrientationTerminator;
import Odometer.SimpleOdometer;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;
@Autonomous
public class MovementTest extends BasicOpmode {
    Vector3 position, velocity;
    SimpleOdometer odometer;
    Registers registers;
    public MovementTest(){
        super(1);
    }
    @Override
    public void setup() {
        robot.enableAll();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("strafe", "40, 0, 0");
        defaults.put("forward", "40, 40, 0");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        registers = new Registers(defaults, defaultTurns);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem system = new MotionSystem(statemachine, odometer, position);
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
                terminate = isStarted();
                stateMachine.activateLogic("Odometry");
            }
        };
        StateMachineManager strafe = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("strafe"), 0.5));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("strafe"), 2);
            }
        };
        StateMachineManager forward = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", system.driveToPoint(registers.getPoint("forward"), 0.5));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, registers.getPoint("forward"), 2);
            }
        };
        stateMachineSwitcher.start(init, strafe, forward);
    }
}
