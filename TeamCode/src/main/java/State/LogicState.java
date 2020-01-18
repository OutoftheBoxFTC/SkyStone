package State;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;

public abstract class LogicState {
    protected StateMachine stateMachine;
    protected String stateName;
    public LogicState(StateMachine stateMachine){
        this.stateMachine = stateMachine;
    }

    public abstract void update(SensorData sensors, HardwareData hardware);

    protected void deactivateThis(){
        stateMachine.deactivateLogic(stateName);
    }

    public void setStateName(String stateName) {
        this.stateName = stateName;
    }

    public void init(SensorData sensors, HardwareData hardware){

    }

    public String getStateName() {
        return stateName;
    }
}