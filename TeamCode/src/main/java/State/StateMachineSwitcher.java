package State;

import java.util.ArrayList;
import java.util.Arrays;

import Hardware.HardwareData;
import Hardware.SensorData;

public class StateMachineSwitcher {
    ArrayList<StateMachineManager> managerList;
    int index = 0;
    public void start(StateMachineManager... managers){
        managerList = new ArrayList<>();
        managerList.addAll(Arrays.asList(managers));
    }
    public void update(SensorData sensors, HardwareData hardware){
        managerList.get(index).update(sensors, hardware);
        if(managerList.get(index).shouldTerminate()){
            managerList.get(index).stop();
            index ++;
        }
    }
}
