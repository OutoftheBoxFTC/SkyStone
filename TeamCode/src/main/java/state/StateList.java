package state;

import java.util.ArrayList;
import java.util.HashMap;

import hardware.ReadData;

public class StateList {
    private ArrayList<Integer> priorities;
    private ArrayList<LogicState> logicStates;
    public StateList(){
        priorities = new ArrayList<>();
        logicStates = new ArrayList<>();
    }

    public void add(HashMap<LogicState, Integer> activatedLogicStates){
        for(LogicState state : activatedLogicStates.keySet()) {
            int priority = activatedLogicStates.get(state);
            int i = 0;
            if(logicStates.size()>0) {
                for (i = 0; i < priorities.size() && priorities.get(i) < priority; i++);
            }
            logicStates.add(i, state);
            priorities.add(i, priority);
        }
    }

    public void remove(ArrayList<LogicState> removedLogicStates){
        for(LogicState state : removedLogicStates){
            if(logicStates.contains(state)){
                int i = logicStates.indexOf(state);
                logicStates.remove(state);
                priorities.remove(i);
            }
        }
    }

    public ArrayList<LogicState> getLogicStates() {
        return logicStates;
    }

    public void update(ReadData data){
        for (int i = logicStates.size()-1; i >= 0 ; i--) {
            logicStates.get(i).update(data);
        }
    }
}
