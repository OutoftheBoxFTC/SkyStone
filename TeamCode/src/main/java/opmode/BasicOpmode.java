package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Debug.Connector;
import State.*;
import Hardware.*;

public abstract class BasicOpmode extends LinearOpMode {
    protected Hardware robot;
    StateMachine statemachine;
    StateMachineSwitcher stateMachineSwitcher;
    double driveLoopIterations, currentLoops;
    boolean debug;
    protected static final double TRANSLATION_FACTOR = (0.0010329132/2);
    public BasicOpmode(double driveLoopIterations){
        this.driveLoopIterations = driveLoopIterations;
        debug = false;
    }

    public BasicOpmode(double driveLoopIterations, boolean debug){
        this.driveLoopIterations = driveLoopIterations;
        this.debug = debug;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        statemachine = new StateMachine();
        robot = new Hardware(this, telemetry);
        stateMachineSwitcher = new StateMachineSwitcher();
        setup();
        robot.init();
        robot.calibrate();
        HardwareData hardware = new HardwareData(System.currentTimeMillis());
        currentLoops = 1;
        while(!isStopRequested()){
            hardware.setTimestamp(System.currentTimeMillis());
            SensorData sensors = robot.update(hardware);
            telemetry.addData("Hardware Latency", System.currentTimeMillis() - hardware.getTimestamp());
            telemetry.addData("Sensors Latency", System.currentTimeMillis() - sensors.getTimestamp());
            telemetry.addData("Active Manager", stateMachineSwitcher.getActiveManager());
            if(debug){
                Connector.getInstance().addTelemetry("Hardware Latency", System.currentTimeMillis() - hardware.getTimestamp());
                Connector.getInstance().addTelemetry("Sensors Latency", System.currentTimeMillis() - sensors.getTimestamp());
                Connector.getInstance().addTelemetry("Active Manager", stateMachineSwitcher.getActiveManager());
            }
            statemachine.update(sensors, hardware);
            if(currentLoops <= 0){
                hardware.setMotorPowers(statemachine.getDriveVelocities(sensors));
                currentLoops = 1;
            }
            stateMachineSwitcher.update(sensors, hardware);
            currentLoops -= (1/driveLoopIterations);
            telemetry.update();
        }
    }

    public abstract void setup();
}
