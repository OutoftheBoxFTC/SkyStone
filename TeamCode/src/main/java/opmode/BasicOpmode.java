package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import State.*;
import Hardware.*;

public abstract class BasicOpmode extends LinearOpMode {
    private Hardware robot;
    StateMachine statemachine;
    StateMachineSwitcher stateMachineSwitcher;
    double driveLoopIterations, currentLoops;
    public BasicOpmode(double driveLoopIterations){
        this.driveLoopIterations = driveLoopIterations;
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
            SensorData sensors = robot.update(hardware);
            statemachine.update(sensors, hardware);
            if(currentLoops <= 0){
                hardware.setMotorPowers(statemachine.getDriveVelocities());
                currentLoops = 1;
            }
            currentLoops -= (1/driveLoopIterations);
        }
    }

    public abstract void setup();

    public abstract void run();
}
