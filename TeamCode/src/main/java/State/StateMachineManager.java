package State;

import java.util.HashMap;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import math.Vector4;

public abstract class StateMachineManager {
    public HashMap<String, LogicState> logicStates;
    public HashMap<String, LogicState> exemptedLogicstates;
    public HashMap<String, DriveState> driveState;
    public boolean terminate;
    public StateMachine stateMachine;
    public StateMachineManager(StateMachine stateMachine){
        this.stateMachine = stateMachine;
        logicStates = new HashMap<>();
        driveState = new HashMap<>();
        exemptedLogicstates = new HashMap<>();
    }

    public abstract void setup();

    public void start(SensorData sensors, HardwareData hardware){
        stateMachine.appendDriveStates(driveState);
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendLogicStates(exemptedLogicstates);
        for(String state : logicStates.keySet()){
            stateMachine.activateLogic(state);
        }
        for(String state : driveState.keySet()){
            stateMachine.setActiveDriveState(state);
        }
        onStart(sensors, hardware);
    }

    public void onStart(SensorData sensors, HardwareData hardware){

    }

    public abstract void update(SensorData sensors, HardwareData hardware);

    public void onStop(SensorData sensors, HardwareData hardware){

    }

    public boolean shouldTerminate(){
        return terminate;
    }

    public void stop(SensorData sensors, HardwareData hardware){
        onStop(sensors, hardware);
        for(LogicState state : logicStates.values()){
            state.deactivateThis();
        }
        for(DriveState state : driveState.values()){
            state.deactivateThis();
        }
    }

    public static StateMachineManager timer(final long time, StateMachine stateMachine){
        return new StateMachineManager(stateMachine) {
            long locTimer = 0;
            @Override
            public void setup() {
                driveState.put("main", new DriveState(stateMachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return Vector4.ZERO();
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {

                    }
                });
                locTimer = System.currentTimeMillis() + time;
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = System.currentTimeMillis() >= locTimer;
            }
        };
    }
}
