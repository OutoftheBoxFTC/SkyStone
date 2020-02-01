package State;

import java.util.ArrayList;
import java.util.Arrays;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;

public class StateMachineSwitcher {
    ArrayList<StateMachineManager> managerList;
    boolean started = false;
    int index = 0;
    public void init(StateMachineManager... managers){
        managerList = new ArrayList<>();
        managerList.addAll(Arrays.asList(managers));
    }
    public void start(SensorData sensors, HardwareData hardware){
        managerList.get(index).setup();
        managerList.get(index).start(sensors, hardware);
        started = true;
    }
    public void update(SensorData sensors, HardwareData hardware){
        managerList.get(index).update(sensors, hardware);
        if(managerList.get(index).shouldTerminate()){
            managerList.get(index).stop(sensors, hardware);
            index ++;
            if(index < managerList.size()) {
                managerList.get(index).setup();
                managerList.get(index).start(sensors, hardware);
            }else{
                index = 0;
                managerList.get(index).setup();
                managerList.get(index).start(sensors, hardware);
            }
        }
    }

    public void setActive(StateMachineManager manager, SensorData sensorData, HardwareData hardware){
        managerList.get(index).stop(sensorData, hardware);
        index = managerList.indexOf(manager);
        managerList.get(index).start(sensorData, hardware);
    }

    public String getActiveManager(){
        return managerList.get(index).getClass().getName();
    }

    public boolean isStarted(){
        return started;
    }
}
