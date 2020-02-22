package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Debug.Connector;
import Debug.Registers;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.CircleCorrectionVector;
import Motion.MotionSystem;
import Motion.Terminator.OrientationTerminator;
import Odometer.SimpleOdometer;
import State.LogicState;
import State.StateMachineManager;
import State.VelocityDriveState;
import math.Vector3;
@TeleOp
public class MovementTest extends BasicOpmode {
    Vector3 position, velocity;
    SimpleOdometer odometer;
    HashMap<String, Vector3> movements = new HashMap<>();
    HashMap<String, Vector3> helperSplines = new HashMap<>();
    public MovementTest(){
        super(1, true);
    }
    @Override
    public void setup() {
        robot.enableAll();
        movements.put("forward", new Vector3(-10, 10, 90));
        helperSplines.put("forward", new Vector3(0, 10, 90));
        final HashMap<String, String> defaultTurns = new HashMap<>();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem system = new MotionSystem(statemachine, position, velocity);
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
                Connector.getInstance().addOrientation(position);
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
        StateMachineManager forward = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("drive", new CircleCorrectionVector(stateMachine, position, movements.get("forward"), helperSplines.get("forward"), 0.5, 10));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, movements.get("forward"), 5);
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

            }
        };
        stateMachineSwitcher.init(init, forward, end);
    }
}
