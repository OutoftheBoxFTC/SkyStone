package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector2;
import math.Vector3;
import motion.DriveToZero;
import motion.PIDControl;
import motion.PIDControl2;
import motion.VelocityDriveState;
import odometry.Odometer;
import odometry.SimpleOdometer;
import state.DriveState;
import state.LogicState;
import state.Orientation;

@TeleOp(name = "Simple Odometry Test")

public class SimpleOdometryTest extends BasicOpmode {
    private Odometer odometer;
    private Vector3 position1, position2, velocity;
    private static final double TRANSLATION_TOLERANCE = 0.1, ROTATION_TOLERANCE = Math.toRadians(0.5);
    private MecanumDrive drive;

    public SimpleOdometryTest() {
        super(0.1, false);
        drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
    }

    @Override
    protected void setup() {
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.registerDevice(Hardware.HardwareDevice.HUB_1_BULK);
        //telemetry.enableLogger();
        odometer = new SimpleOdometer();
        position1 = new Vector3(0, 0, 0);
        position2 = new Vector3(0, 0, 0);
        velocity = new Vector3(0, 0, 0);

        HashMap<String, LogicState> logicStates = new HashMap<>();
        final HashMap<String, DriveState> driveStates = new HashMap<>();
        logicStates.put("Orientation", new Orientation(stateMachine, odometer, position1, velocity){
            @Override
            public void update(ReadData data) {
                super.update(data);
                telemetry.setHeader("X1", position1.getA());
                telemetry.setHeader("Y1", position1.getB());
                telemetry.setHeader("R1", Math.toDegrees(position1.getC()));
            }
        });
        logicStates.put("Init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    stateMachine.activateLogic("Orientation");
                    stateMachine.activateLogic("Tracking");
                    deactivateThis();
                }
            }
        });

        logicStates.put("Tracking", new LogicState(stateMachine) {
            @Override
            public void init(ReadData data) {

            }

            @Override
            public void update(ReadData data) {
                if(gamepad1.a.isActive()&&gamepad1.a.isUpdated()){
                    deactivateThis();
                    stateMachine.activateLogic("Terminate At Zero");
                    stateMachine.setActiveDriveState("Drive To Zero");
                }
            }
        });

        logicStates.put("Terminate At Zero", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(new Vector2(position1).length()<0.1&& position1.getC()< Math.toRadians(0.1)){
                    stateMachine.setActiveDriveState("None");
                    deactivateThis();
                }
            }
        });

        driveStates.put("None", new VelocityDriveState(stateMachine, drive) {
            @Override
            public Vector3 getRobotVelocity() {
                return new Vector3(0, 0, 0);
            }
        });

        //TODO tune these and create some kind of global tuning reference system
        driveStates.put("Drive To Zero", new DriveToZero(position1, new PIDControl2(1, 1, 1), new PIDControl(1, 1, 1), stateMachine, drive));
        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(logicStates);
        stateMachine.setActiveDriveState("None");
        stateMachine.activateLogic("Init");
    }
}
