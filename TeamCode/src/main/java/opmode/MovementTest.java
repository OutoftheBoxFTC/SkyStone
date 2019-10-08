package opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Odometer.Odometer;
import Odometer.SimpleOdometer;
import drivetrain.MecanumDrive;
import hardware.Hardware;
import hardware.ReadData;
import math.Vector3;
import motion.DriveToPointBuilder;
import motion.Terminator.Terminator;
import Odometer.DumbSimpleOdometer;
import state.DriveState;
import state.LogicState;
import state.Orientation;
import state.StateMachine;
import state.motion.CorrectionVector;
import state.motion.TurnCorrectionVector;

@TeleOp
public class MovementTest extends BasicOpmode {
    protected static final double TRANSLATION_FACTOR = 0.0010329132;
    public MovementTest() {
        super(0.3, false);
    }

    @Override
    protected void setup() {
        final SimpleOdometer odometer;
        final Vector3 position, velocity;
        robot.enableDevice(Hardware.HardwareDevice.HUB_1_BULK);
        robot.enableDevice(Hardware.HardwareDevice.DRIVE_MOTORS);
        robot.enableDevice(Hardware.HardwareDevice.GYRO);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity, telemetry);
        DriveToPointBuilder builder = new DriveToPointBuilder(stateMachine, new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1));
        DriveState firstMovement = new CorrectionVector(stateMachine, new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1), velocity, new Vector3(0, 18, 0), 0, Terminator.NONE(), 0.2, odometer);
        DriveState firstRotation = new TurnCorrectionVector(stateMachine, new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1), 0.01, 270, 1, position, "Move");
        HashMap<String, DriveState> driveStates = new HashMap<>();
        HashMap<String, LogicState> states = new HashMap<>();
        states.put("Orientation", new LogicState(stateMachine){
            @Override
            public void update(ReadData data) {
                odometer.update(data);
                telemetry.setHeader("x", position.getA());
                telemetry.setHeader("y", position.getB());
                telemetry.setHeader("r", position.getC());
            }
        });
        states.put("init", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                if(opModeIsActive()){
                    stateMachine.activateLogic("Tracking");
                    stateMachine.setActiveDriveState("Move");
                    stateMachine.activateLogic("Orientation");
                    odometer.start(data);
                    deactivateThis();
                }
            }
        });
        states.put("Tracking", new LogicState(stateMachine) {
            @Override
            public void update(ReadData data) {
                telemetry.setHeader("Position", position);
                telemetry.setHeader("Velocity", velocity);
                telemetry.setHeader("raw", new Vector3(data.getLeft(), data.getRight(), data.getAux()));
            }
        });
        driveStates.put("Move", firstMovement);
        //driveStates.put("Rotate", firstRotation);
        stateMachine.appendDriveStates(driveStates);
        stateMachine.appendLogicStates(states);
        stateMachine.activateLogic("init");
    }
}
