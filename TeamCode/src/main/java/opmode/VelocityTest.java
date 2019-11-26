package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import state.motion.FieldCentricDriverControl;
import odometry.SimpleOdometer;
import state.DriveState;
import state.LogicState;
import state.Orientation;

@TeleOp(name = "VelocityTest")
public class VelocityTest extends BasicOpmode {
    private Vector3 position;
    private Vector3 velocity;
    private MecanumDrive drive;
    private double vR;
    public VelocityTest() {
        super(0.1, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.HUB_2_BULK)
                .registerDevice(Hardware.HardwareDevice.HUB_1_BULK)
                .registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);

        HashMap<String, DriveState> driveStates = new HashMap<>();
        HashMap<String, LogicState> logicStates = new HashMap<>();
        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.activateLogic("orientation");
                    stateMachine.setActiveDriveState("controller");
                    stateMachine.activateLogic("run");
                }
            }
        });

        logicStates.put("orientation", new Orientation(stateMachine, new SimpleOdometer(), position, velocity));
        logicStates.put("run", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(gamepad1.a.isActive()&&gamepad1.a.isUpdated()){
                    deactivateThis();
                    stateMachine.activateLogic("track");
                }
                telemetry.setHeader("vR", vR);
            }
        });
        logicStates.put("track", new LogicState(stateMachine) {
            private double initialR;
            private double previousR;
            private boolean flipped;
            private long start;
            @Override
            public void init(ReadData data) {
                super.init(data);
                initialR = position.getC();
                previousR = initialR;
                flipped = false;
                start = data.getHub1BulkTime();
            }

            @Override
            public void update(ReadData data) {
                if(position.getC() < previousR){
                    flipped = true;
                }
                if(flipped&&position.getC()>initialR){
                    long dt = data.getHub1BulkTime()-start;
                    vR = (Math.PI*2+(position.getC()-initialR))/((double)dt)*1.0e9;
                    deactivateThis();
                    stateMachine.activateLogic("run");
                }
                previousR = position.getC();
            }
        });
        driveStates.put("controller", new FieldCentricDriverControl(position, gamepad1, stateMachine, drive));
        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(logicStates);
        stateMachine.activateLogic("init");

    }
}
