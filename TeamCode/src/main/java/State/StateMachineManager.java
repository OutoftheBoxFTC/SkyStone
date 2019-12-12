package State;

import java.util.HashMap;

import Hardware.HardwareData;
import Hardware.SensorData;

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

    public void start(){
        stateMachine.appendDriveStates(driveState);
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendLogicStates(exemptedLogicstates);
        for(String state : logicStates.keySet()){
            stateMachine.activateLogic(state);
        }
        for(String state : driveState.keySet()){
            stateMachine.setActiveDriveState(state);
        }
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
}
