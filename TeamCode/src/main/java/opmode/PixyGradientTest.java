package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import HardwareSystems.Hardware;
import State.LogicState;
import State.StateMachineManager;

@TeleOp
public class PixyGradientTest extends BasicOpmode{
    int y = 0;
    boolean[] map = new boolean[42];
    int[] byteMap = new int[42];
    public PixyGradientTest() {
        super(0);
    }

    @Override
    public void setup() {
        robot.enableAll();
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
                        for(int i = 0; i < 16; i ++){
                            pos3 += byteMap[i];
                        }
                        pos3 = pos3 / 16;
                        for(int i = 16; i < 30; i ++){
                            pos2 += byteMap[i];
                        }
                        pos2 = pos2 / 14;
                        for(int i = 30; i < 42; i ++){
                            pos1 += byteMap[i];
                        }
                        pos1 = pos1 / 12;
                        int max = Math.min(Math.min(pos1, pos2), pos3);
                        telemetry.addData("Max1", pos3);
                        telemetry.addData("Max2", pos2);
                        telemetry.addData("Max3", pos1);
                        telemetry.addData("Position", max == pos1 ? "1" : (max == pos2 ? "2" : "3"));
                        pos1 = 0;
                        pos2 = 0;
                        pos3 = 0;
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {

            }
        };
        stateMachineSwitcher.start(init, main);
    }
}
