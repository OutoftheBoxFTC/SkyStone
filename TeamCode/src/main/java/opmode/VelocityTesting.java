package opmode;


import java.util.HashMap;

import Debug.Registers;
import HardwareSystems.Hardware;
import HardwareSystems.*;
import Motion.MotionSystem;
import Motion.Terminator.OrientationTerminator;
import Odometer.SimpleOdometer;
import State.LogicState;
import State.StateMachineManager;
import math.Vector3;

public class VelocityTesting extends BasicOpmode{
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;
    Registers registers;
    public VelocityTesting() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.enableDevice(Hardware.HardwareDevices.LEFT_PIXY);
        firstSkystone = Vector3.ZERO();
        HashMap<String, String> defaults = new HashMap<>();
        defaults.put("moveToSkystones", "-10, 0, 0");
        defaults.put("scanSkystones", "0, -24, 0");
        defaults.put("moveToFoundation", "-10, 24, 90");
        final HashMap<String, String> defaultTurns = new HashMap<>();
        defaultTurns.put("rotateToGrabSkystone", "0, 0, 80");
        registers = new Registers(defaults, defaultTurns);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        final MotionSystem system = new MotionSystem(statemachine, position, velocitySystem);
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
            }

            @Override
            public void onStop(SensorData sensors, HardwareData hardware){
                stateMachine.activateLogic("Odometry");
            }
        };
        StateMachineManager firstMovement = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("driveToPoint", system.driveToPoint(new Vector3(0, 15, 0), 1));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, new Vector3(0, 15, 0), 3);
            }
        };
        StateMachineManager secondMovement = new StateMachineManager(statemachine) {
            @Override
            public void setup() {
                driveState.put("driveToPoint", system.driveToPoint(new Vector3(0, 0, 0), 1));
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = OrientationTerminator.shouldTerminatePosition(position, new Vector3(0, 0, 0), 3);
            }
        };
        stateMachineSwitcher.start(init, firstMovement, secondMovement);
    }
}
