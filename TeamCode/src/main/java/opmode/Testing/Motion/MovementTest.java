package opmode.Testing.Motion;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Debug.Connector;
import Debug.Registers;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.CircleCorrectionVector;
import Motion.CurveBuilder;
import Motion.MotionSystem;
import Motion.Terminator.OrientationTerminator;
import Odometer.SimpleOdometer;
import State.LogicState;
import State.StateMachineManager;
import State.VelocityDriveState;
import math.Vector2;
import math.Vector3;
import opmode.BasicOpmode;

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
        final HashMap<String, String> defaultTurns = new HashMap<>();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem system = new MotionSystem(statemachine, position, velocity);
        final CurveBuilder curveBuilder = new CurveBuilder(statemachine, position);
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
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware) {
                stateMachine.activateLogic("Odometry");
            }
        };
        StateMachineManager forward = new StateMachineManager(statemachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void setup() {
                driveState.put("main", curveBuilder.newCurve()
                        .setSpline(new Vector2(0, 10))
                        .setSpline(new Vector2(40, 10))
                        .setSpeed(1)
                        .setAngle(110)
                        .setMinPower(0.5)
                        .complete());
                orientationTerminator = new OrientationTerminator(position, new Vector3(40, 10, 0), 2, 1);
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = orientationTerminator.shouldTerminate(sensors);
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
                telemetry.addData("Error", position.getVector2().distanceTo(new Vector2(20, 15)));
            }
        };
        stateMachineSwitcher.init(init, forward, end);
    }
}
