package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import HardwareSystems.HardwareConstants;
import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import HardwareSystems.Hardware;
import State.LogicState;
import State.StateMachineManager;

@TeleOp
public class PixyGradientTest extends BasicOpmode{
    int skystonePos;
    public PixyGradientTest() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
        robot.disableDevice(Hardware.HardwareDevices.SIDE_LASERS);
        robot.enableDevice(Hardware.HardwareDevices.RIGHT_PIXY);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addLine("Press Start to Start. I hope that should have been obvious");
                terminate = isStarted();
            }
        };
        StateMachineManager main = new StateMachineManager(statemachine) {
            int y = 0;
            boolean[] map = new boolean[42];
            int[] byteMap = new int[42];
            @Override
            public void setup() {
                logicStates.put("main", new LogicState(statemachine) {
                    int counter = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(y > 207){
                            y = 0;
                            counter = 0;
                        }
                        byte[] tmp = robot.getPixy().getCoordinateColor(157, y);
                        byteMap[counter] = Math.abs(tmp[tmp.length-2] & 0xFF);
                        y += 5;
                        counter ++;
                        if(y == 210){
                            y = 207;
                            counter = 41;
                        }
                    }
                });
                logicStates.put("readout", new LogicState(statemachine) {
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        for(int i = 0; i < map.length; i ++){
                            telemetry.addData("Position " + (i), byteMap[i]);
                        }
                    }
                });
                logicStates.put("result", new LogicState(statemachine) {
                    int pos1 = 0, pos2 = 0, pos3 = 0;
                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        for(int i = 0; i < 10; i ++){
                            if(byteMap[i] < 255){
                                pos3 ++;
                            }
                        }
                        for(int i = 10; i < 20; i ++){
                            if(byteMap[i] < 255){
                                pos2 ++;
                            }
                        }
                        for(int i = 26; i < 32; i ++){
                            if(byteMap[i] < 255){
                                pos1 ++;
                            }
                        }
                        int max = Math.max(Math.max(pos1, pos2), pos3);
                        telemetry.addData("Max1", pos3);
                        telemetry.addData("Max2", pos2);
                        telemetry.addData("Max3", pos1);
                        telemetry.addData("Position", max == pos1 ? "3" : (max == pos2 ? "2" : "1"));
                        skystonePos = (max == pos1 ? 3 : (max == pos2 ? 2 : 1));
                        telemetry.addData("SkystonePos", skystonePos);
                        double conf1 = pos3;
                        double conf2 = pos2;
                        double conf3 = pos1;
                        double totConf = conf1 + conf2 + conf3;
                        telemetry.addData("Confidence 1", (conf1/totConf)*100);
                        telemetry.addData("Confidence 2", (conf2/totConf)*100);
                        telemetry.addData("Confidence 3", (conf3/totConf)*100);
                        pos1 = 0;
                        pos2 = 0;
                        pos3 = 0;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addData("Debug", "Got to this point");
                RobotLog.i("We are in the update loop");
            }
        };
        stateMachineSwitcher.init(init, main);
    }
}
