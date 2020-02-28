package opmode.Testing.Motion;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.HashMap;

import HardwareSystems.HardwareData;
import HardwareSystems.SensorData;
import Motion.MecanumSystem;
import Odometer.SimpleOdometer;
import State.DriveState;
import State.LogicState;
import State.StateMachineManager;
import math.Vector2;
import math.Vector3;
import math.Vector4;
import opmode.BasicOpmode;

@TeleOp
public class VelocityLimitsTest extends BasicOpmode {
    SimpleOdometer odometer;
    Vector3 position, velocity, firstSkystone;

    public VelocityLimitsTest() {
        super(1);
    }

    @Override
    public void setup() {
        robot.enableAll();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new SimpleOdometer(TRANSLATION_FACTOR, position, velocity);
        HashMap<String, LogicState> nonManagedLogicStates = new HashMap<>();
        nonManagedLogicStates.put("odometry", new LogicState(statemachine) {
            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                odometer.update(sensors);
            }
        });
        statemachine.appendLogicStates(nonManagedLogicStates);
        StateMachineManager init = new StateMachineManager(statemachine) {
            @Override
            public void setup() {

            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                terminate = isStarted();
            }

            public void onStop(SensorData sensors, HardwareData hardware){
                odometer.start(sensors);
                stateMachine.activateLogic("odometry");
            }
        };
        StateMachineManager main = new StateMachineManager(statemachine) {
            Vector3 maxVel = Vector3.ZERO();
            @Override
            public void setup() {
                driveState.put("main", new DriveState(statemachine) {
                    @Override
                    public Vector4 getWheelVelocities(SensorData sensors) {
                        return MecanumSystem.translate(new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
                    }

                    @Override
                    public void update(SensorData sensors, HardwareData hardware) {
                        if(gamepad1.right_trigger > 0.5) {
                            mixer.playHorn();
                        }else{
                            mixer.stopAll();
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                telemetry.addData("Position", position);
                telemetry.addData("Velocity", velocity);
                if(new Vector2(maxVel.getA(), maxVel.getB()).distanceTo(Vector2.ZERO()) < new Vector2(velocity.getA(), velocity.getB()).distanceTo(Vector2.ZERO())){
                    maxVel.set(velocity);
                }
                RobotLog.ii("Velocity", velocity.toString());
                telemetry.addData("Max Velocity", maxVel);
            }
        };
        stateMachineSwitcher.init(init, main);
    }
}
