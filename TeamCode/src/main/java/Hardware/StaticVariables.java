package Hardware;

import State.StateMachine;

public class StaticVariables {
    private static StaticVariables instance = new StaticVariables();
    private CalibrationSystem calibration;
    public StaticVariables(){ }
    public void setCalibration(CalibrationSystem calibration){
        this.calibration = calibration;
    }
    public CalibrationSystem getCalibration(){
        return calibration;
    }
    public static StaticVariables getInstance(){
        return instance;
    }
}
