package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import debug.FPSDebug;
import debug.SmartTelemetry;
import hardware.Hardware;
import hardware.ReadData;
import hardware.controller.SmartGamepad;
import math.Vector4;
import state.StateMachine;

/**
 * Notes:
 *  "Dynamics" refers to any robot centric metric
 *  Velocity is obtained from odometry wheel readings rather than position differentiation (for now)
 */
public abstract class BasicOpmode extends LinearOpMode {
    protected Hardware robot;
    protected FPSDebug fpsDebug;
    protected SmartTelemetry telemetry;
    protected SmartGamepad gamepad1, gamepad2;
    protected StateMachine stateMachine;
    private ExecutorService threadManager;

    private double driveLoopPriority;

    protected final boolean debug;

    public BasicOpmode(double driveLoopPriority, boolean debug){
        this.driveLoopPriority = Math.min(1, driveLoopPriority);
        this.debug = debug;
    }

    @Override
    public void runOpMode() {
        this.telemetry = new SmartTelemetry(super.telemetry);
        fpsDebug = new FPSDebug(telemetry, "Main Loop");
        stateMachine = new StateMachine();
        if (!debug) {
            robot = new Hardware(this, telemetry);
            threadManager = Executors.newFixedThreadPool(1);
        }
        gamepad1 = new SmartGamepad(super.gamepad1);
        gamepad2 = new SmartGamepad(super.gamepad2);
        setup();
        if (!debug){
            RobotLog.i("We are not in Debug Mode");
            robot.init();
            robot.calibrate();
            threadManager.execute(robot);
        }
        double driveIterations = 0;
        while (!isStopRequested()){
            ReadData data = null;
            if(!debug) {
                data = robot.newData();//stalls here until hardware loop obtains new data
            }
            fpsDebug.startIncrement();
            gamepad1.update();
            gamepad2.update();
            stateMachine.update(data);
            while (driveIterations >= 1) {
                Vector4 wheels = stateMachine.getDriveVelocities();
                robot.drive(wheels.getA(), wheels.getB(), wheels.getC(), wheels.getD());
                driveIterations--;
            }
            driveIterations += driveLoopPriority;
            fpsDebug.endIncrement();
            fpsDebug.update();
            fpsDebug.queryFPS();
            telemetry.setHeader("Activated Logic States", Arrays.deepToString(stateMachine.getActiveLogicStates()));
            telemetry.update();
        }
        try {
            telemetry.stop();
        } catch (IOException e) {
            e.printStackTrace();
        }
        threadManager.shutdown();
    }

    protected abstract void setup();
}