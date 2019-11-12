package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import hardware.Hardware;
import hardware.ReadData;
import math.Vector4;
import opencv.DetectionPipeline;
import state.DriveState;
import state.LogicState;

@TeleOp(name = "Webcam Test")
public class WebcamTest extends BasicOpmode {

    public WebcamTest() {
        super(0, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.WEBCAM_1);
        final DetectionPipeline pipeline = new DetectionPipeline();

        robot.setPipeline(pipeline);
        HashMap<String, LogicState> logicStates = new HashMap<>();

        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.activateLogic("detect");
                }
            }
        });
        logicStates.put("detect", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("position", pipeline.getDetectedPosition());
            }
        });

        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("zero", new DriveState(stateMachine) {
            @Override
            public Vector4 getWheelVelocities() {
                return Vector4.ZERO();
            }
        });
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);
        stateMachine.activateLogic("init");
    }
}
