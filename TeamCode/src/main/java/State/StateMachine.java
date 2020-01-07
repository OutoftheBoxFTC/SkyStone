package State;

import java.util.ArrayList;
import java.util.HashMap;

import Hardware.HardwareData;
import Hardware.SensorData;
import math.Vector4;

public class StateMachine {
    private HashMap<String, DriveState> driveStates;
    private HashMap<String, LogicState> logicStates;

    private DriveState activeDriveState, activatedDriveState;
    private ArrayList<LogicState> activeLogicStates;

    private ArrayList<LogicState> deactivatedLogicStates, activatedLogicStates;

    private HashMap<String, Long> queriedActivations;

    public StateMachine(){
        driveStates = new HashMap<>();
        logicStates = new HashMap<>();

        activeLogicStates = new ArrayList<>();
        activatedLogicStates = new ArrayList<>();
        activeDriveState = null;
        activatedDriveState = null;
        activatedLogicStates = new ArrayList<>();
        deactivatedLogicStates = new ArrayList<>();
        queriedActivations = new HashMap<>();
    }

    public void update(SensorData sensors, HardwareData hardware){
        for(LogicState state : activatedLogicStates){
            state.init(sensors, hardware);
        }
        activeLogicStates.addAll(activatedLogicStates);
        activatedLogicStates.clear();
        for(LogicState state : activeLogicStates){
            state.update(sensors, hardware);
        }

        if(!queriedActivations.isEmpty()) {
            ArrayList<String> removedQueries = new ArrayList<>();
            for (String key : queriedActivations.keySet()) {
                if (queriedActivations.get(key) < System.currentTimeMillis()) {
                    activatedLogicStates.add(logicStates.get(key));
                    removedQueries.add(key);
                }
            }
            for(String key : removedQueries){
                queriedActivations.remove(key);
            }
        }

        if(activatedDriveState != null){
            activeDriveState = activatedDriveState;
            activatedDriveState = null;
        }

        activeLogicStates.removeAll(deactivatedLogicStates);
        deactivatedLogicStates.clear();
    }

    public Vector4 getDriveVelocities(SensorData sensors){
        Vector4 velocity = new Vector4(0, 0, 0, 0);
        if(activeDriveState!=null){
            velocity = activeDriveState.getWheelVelocities(sensors);
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
        activatedLogicStates.add(logicStates.get(state));
    }

    public void setActiveDriveState(String state){
        if(activatedDriveState == null){
            activatedDriveState = driveStates.get(state);
            activateLogic(state);
        }
        else {
            if(!activatedDriveState.getStateName().equals(state)){
                deactivatedLogicStates.add(driveStates.get(state));
                activatedDriveState = driveStates.get(state);
                activateLogic(state);
            }
        }
    }

    public boolean contains(LogicState state){
        return logicStates.containsValue(state);
    }

    public boolean contains(String state){
        return logicStates.containsKey(state);
    }

    public LogicState getLogicState(String logicState){
        return logicStates.get(logicState);
    }

    public boolean logicStateActive(String logicState){
        return activeLogicStates.contains(logicStates.get(logicState));
    }

    public boolean driveStateIsActive(String driveState){
        return activeDriveState.equals(driveStates.get(driveState));
    }

    public void activateLogic(String state, long delay) {
        //TODO finish me
    }

    public String[] getActiveLogicStates() {
        ArrayList<String> activeStates = new ArrayList<>();
        for(LogicState state : activeLogicStates){
            activeStates.add(state.getStateName());
        }
        return activeStates.toArray(new String[activatedLogicStates.size()]);
    }
}