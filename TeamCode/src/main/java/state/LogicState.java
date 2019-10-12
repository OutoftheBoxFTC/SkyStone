package state;

import hardware.ReadData;

public abstract class LogicState {
    protected StateMachine stateMachine;
    protected String stateName;
    public LogicState(StateMachine stateMachine){
        this.stateMachine = stateMachine;
    }

    public abstract void update(ReadData data);

    protected void deactivateThis(){
        stateMachine.deactivateLogic(stateName);
    }

    public void setStateName(String stateName) {
        this.stateName = stateName;
    }

    public void init(ReadData data){

    }

    public String getStateName() {
        return stateName;
    }
}