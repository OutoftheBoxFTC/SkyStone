package State;

import java.util.ArrayList;
import java.util.Arrays;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;

public class StateMachineSwitcher {
    ArrayList<StateMachineManager> managerList;
    int index = 0;
    public void start(StateMachineManager... managers){
        managerList = new ArrayList<>();
        managerList.addAll(Arrays.asList(managers));
        managerList.get(index).setup();
        managerList.get(index).start();
    }
    public void update(SensorData sensors, HardwareData hardware){
        managerList.get(index).update(sensors, hardware);
        if(managerList.get(index).shouldTerminate()){
            managerList.get(index).stop(sensors, hardware);
            index ++;
            if(index < managerList.size()) {
                managerList.get(index).setup();
                managerList.get(index).start();
            }else{
                index = 0;
                managerList.get(index).setup();
                managerList.get(index).start();
            }
        }
    }

    public void setActive(StateMachineManager manager, SensorData sensorData, HardwareData hardware){
        managerList.get(index).stop(sensorData, hardware);
        index = managerList.indexOf(manager);
        managerList.get(index).start();
    }

    public String getActiveManager(){
        return managerList.get(index).getClass().getName();
    }
}
