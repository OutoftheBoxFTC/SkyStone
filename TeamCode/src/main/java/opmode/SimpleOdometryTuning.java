package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import math.Vector4;
import state.motion.VelocityDriveState;
import odometry.Odometer;
import odometry.SimpleOdometer;
import state.DriveState;
import state.LogicState;

@TeleOp(name = "Simple Odometer Tuning")
public class SimpleOdometryTuning extends BasicOpmode {

    private Odometer odometer;
    private static final double TAU = Math.PI*2;
    private MecanumDrive drive;
    public SimpleOdometryTuning() {
        super(0.1, false);
        drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
    }

    @Override
    protected void setup() {
        odometer = new SimpleOdometer(1, 1, 1);
        final HashMap<String, LogicState> logicStates = new HashMap<>();
        robot.registerDevice(Hardware.HardwareDevice.GYRO);
        robot.registerDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.registerDevice(Hardware.HardwareDevice.HUB_1_BULK);
        //telemetry.enableLogger();
        logicStates.put("init", new LogicState(stateMachine) {
            @Override
            public void init(ReadData data) {
                telemetry.setHeader("Instructions", "Hit start, then wait for the robot to finish rotating. Note down the generated factors");
            }

            @Override
            public void update(ReadData data) {
                if(isStarted()){
                    deactivateThis();
                    stateMachine.activateLogic("rotation tracking");
                    stateMachine.setActiveDriveState("rotation");
                    telemetry.clearAllHeadersExcept("Main Loop FPS", "Hardware FPS", "Activated Logic States");
                }
            }
        });
        logicStates.put("rotation tracking", new LogicState(stateMachine) {
            private int rotations;
            private double previousGyro;

            @Override
            public void init(ReadData data) {
                rotations = 0;
                previousGyro = 0;
            }

            @Override
            public void update(ReadData data) {
                double gyro = data.getGyro();
                if(rotations >= 10){
                    stateMachine.setActiveDriveState("none");
                    deactivateThis();
                    stateMachine.activateLogic("Stop Moving");
                }
                odometer.updateRobotDynamics(data);
                telemetry.setHeader("rotations", String.valueOf(rotations));
                telemetry.setHeader("gyro", String.valueOf(gyro));
                telemetry.setHeader("position", new Vector3(0, 0, Math.toDegrees(gyro)));
                if(gyro < previousGyro){
                    rotations++;
                }
                previousGyro = gyro;
            }
        });
        logicStates.put("Stop Moving", new LogicState(stateMachine){
            private long lastTime;
            @Override
            public void init(ReadData data) {
                lastTime = System.currentTimeMillis();
            }

            @Override
            public void update(ReadData data) {
                odometer.updateRobotDynamics(data);
                if(System.currentTimeMillis()-lastTime>250){
                    telemetry.setHeader("Instructions", "Note down these factors and"+
                            "Move the robot to a place where you can push it in one direction for a long distance. Hit (A) when you are ready to proceed.");
                    double radiansTurned = 10*TAU+data.getGyro();
                    double rotation = odometer.getGlobalDynamics().getC();
                    double rotationFactor = radiansTurned/rotation;
                    double auxRotationFactor = data.getAux()/rotation;
                    telemetry.setHeader("Rotation Factor", String.valueOf(rotationFactor));
                    telemetry.setHeader("Aux Rotation Factor", String.valueOf(auxRotationFactor));
                    deactivateThis();
                    stateMachine.activateLogic("Wait for A");
                }
            }
        });
        logicStates.put("Wait for A", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(gamepad1.a.isPressed()){
                    deactivateThis();
                    stateMachine.activateLogic("Odometer Readout");
                }
            }
        });
        logicStates.put("Odometer Readout", new LogicState(stateMachine) {
            @Override
            public void init(ReadData data) {
                telemetry.setHeader("Instructions", "Push the robot as far as you can in one cardinal direction. Be extremely careful not to rotate it while doing this.");
            }

            @Override
            public void update(ReadData data) {
                telemetry.setHeader("position", String.valueOf(0.004 * ((data.getLeft() + data.getRight())/2)));
                telemetry.setHeader("LR Avg", String.valueOf((data.getLeft()+data.getRight())/2.0));
                telemetry.setHeader("Aux", String.valueOf(data.getAux()));
            }
        });

        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("none", new DriveState(stateMachine) {
            @Override
            public Vector4 getWheelVelocities() {
                return new Vector4(0, 0, 0, 0);
            }
        });
        driveStates.put("rotation", new VelocityDriveState(stateMachine, drive) {
            @Override
            public Vector3 getRobotVelocity() {
                return new Vector3(0, 0, 0.2);
            }
        });
        stateMachine.appendLogicStates(logicStates);
        stateMachine.appendDriveStates(driveStates);
        stateMachine.activateLogic("init");
        stateMachine.setActiveDriveState("none");
    }
}
