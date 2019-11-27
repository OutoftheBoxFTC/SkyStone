package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.MathUtil;
import math.Vector2;
import math.Vector3;
import odometry.SimpleOdometer;
import state.DriveState;
import state.LogicState;
import state.Orientation;
import state.motion.FieldCentricDriverControl;

@TeleOp(name = "Center Offset Test")
public class CenterOffsetTest extends BasicOpmode {

    public CenterOffsetTest() {
        super(1, false);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.HUB_1_BULK)
                .registerDevice(Hardware.HardwareDevice.HUB_2_BULK)
                .registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);

        final Vector3 position = new Vector3(0, 0, Math.PI/2), actingPosition = new Vector3(position);
        SimpleOdometer odometer = new SimpleOdometer();
        MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);

        HashMap<String, LogicState> logicStates = new HashMap<>();
        HashMap<String, DriveState> driveStates = new HashMap<>();
        final Vector2 offset = Vector2.ZERO();
        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.activateLogic("orientation", 0);
                    stateMachine.activateLogic("run");
                    stateMachine.setActiveDriveState("controller");
                }
            }
        });

        logicStates.put("orientation", new Orientation(stateMachine, odometer, actingPosition, position){
            @Override
            public void update(ReadData data) {
                super.update(data);
                telemetry.setHeader("position", position);
                telemetry.setHeader("acting", actingPosition);
            }
        });

        logicStates.put("run", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(gamepad1.a.isPressed()&&gamepad1.a.isUpdated()){
                    deactivateThis();
                    stateMachine.activateLogic("track");
                }
                telemetry.setHeader("offset", offset);
            }
        });

        logicStates.put("track", new LogicState(stateMachine) {
            private Vector3 initialPose, previousPose;
            private double targetRot;
            @Override
            public void init(ReadData data) {
                super.init(data);
                initialPose = new Vector3(position);
                previousPose = new Vector3(position);
                double TAU = Math.PI*2;
                targetRot = (((position.getC()+Math.PI/4)%TAU)+TAU)%TAU;
            }

            @Override
            public void update(ReadData data) {
                if(position.getC()>targetRot&&previousPose.getC()<targetRot){
                    deactivateThis();
                    stateMachine.activateLogic("run");
                    double angleChange = MathUtil.angleDelta(initialPose.getC(), position.getC());
                    double chordLength = new Vector2(initialPose).distanceTo(new Vector2(position));
                    double radius = chordLength/Math.sin(angleChange/2)/2;
                    double k = radius*Math.cos(angleChange/2);
                    double angleTo = new Vector2(initialPose).angleTo(new Vector2(position));
                    offset.set(MathUtil.rotationMatrix(angleTo).transform(new Vector2(chordLength/2, k)));
                }
                previousPose.set(position);
            }
        });

        driveStates.put("controller", new FieldCentricDriverControl(actingPosition, gamepad1, stateMachine, drive));
    }
}
