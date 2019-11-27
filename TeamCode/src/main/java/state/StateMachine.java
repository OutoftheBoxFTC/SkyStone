package state;

import java.util.ArrayList;
import java.util.HashMap;

import hardware.ReadData;
import math.Vector4;

public class StateMachine {
    private HashMap<String, DriveState> driveStates;
    private HashMap<String, LogicState> logicStates;

    private DriveState activeDriveState, activatedDriveState;
    private StateList activeLogicStates;

    private HashMap<LogicState, Integer> activatedLogicStates;
    private ArrayList<LogicState> deactivatedLogicStates;


    public StateMachine(){
        driveStates = new HashMap<>();
        logicStates = new HashMap<>();
        activeLogicStates = new StateList();
        activatedLogicStates = new HashMap<>();
        activeDriveState = null;
        activatedDriveState = null;
        deactivatedLogicStates = new ArrayList<>();
    }

    public void update(ReadData data){
        for(LogicState state : activatedLogicStates.keySet()){
            state.init(data);
        }

        activeLogicStates.add(activatedLogicStates);
        activatedLogicStates.clear();

        activeLogicStates.update(data);

        if(activatedDriveState != null){
            activeDriveState = activatedDriveState;
            activatedDriveState = null;
        }
        activeLogicStates.remove(deactivatedLogicStates);
        deactivatedLogicStates.clear();
    }

    public Vector4 getDriveVelocities(){
        Vector4 velocity = new Vector4(0, 0, 0, 0);
        if(activeDriveState!=null){
            velocity = activeDriveState.getWheelVelocities();
        }
        return velocity;
    }

    public void appendLogicStates(HashMap<String, ? extends LogicState> states){
        for (String key : states.keySet()){
            states.get(key).setStateName(key);
        }
        logicStates.putAll(states);
    }

    public void appendDriveStates(HashMap<String, DriveState> states){
        driveStates.putAll(states);
        appendLogicStates(states);
    }

    public void deactivateLogic(String state){
        deactivatedLogicStates.add(logicStates.get(state));
    }

    public void activateLogic(String state){
        activateLogic(state, Integer.MAX_VALUE);
    }

    public void activateLogic(String state, int priority){
        activatedLogicStates.put(logicStates.get(state), priority);
    }

    public void setActiveDriveState(String state){
        if(activeDriveState == null){
            activatedDriveState = driveStates.get(state);
            activateLogic(state);
        }
        else {
            if(!activeDriveState.getStateName().equals(state)){
                deactivateLogic(activeDriveState.stateName);
                activatedDriveState = driveStates.get(state);
                activateLogic(state);
            }
        }
    }

    public LogicState getLogicState(String logicState){
        return logicStates.get(logicState);
    }

    public boolean logicStateActive(String logicState){
        return activeLogicStates.getLogicStates().contains(logicStates.get(logicState));
    }

    public boolean driveStateIsActive(String driveState){
        return activeDriveState.equals(driveStates.get(driveState));
    }

    public String[] getActiveLogicStates() {
        ArrayList<String> activeStates = new ArrayList<>();
        for(LogicState state : activeLogicStates.getLogicStates()){
            activeStates.add(state.getStateName());
        }
        return activeStates.toArray(new String[activatedLogicStates.size()]);
    }
}