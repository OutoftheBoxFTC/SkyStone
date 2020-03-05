package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.json.JSONException;

import java.io.IOException;
import java.util.HashMap;

import Debug.Connector;
import Motion.VelocitySystem;
import State.*;
import HardwareSystems.*;
import math.Vector3;

public abstract class BasicOpmode extends LinearOpMode {
    protected Hardware robot;
    private Thread robotThread;
    public StateMachine statemachine;
    public StateMachineSwitcher stateMachineSwitcher;
    private double driveLoopIterations;
    private boolean debug;
    public static final double TRANSLATION_FACTOR = (0.0010329132/2);
    private long timer = 0;
    public SoundMixer mixer;
    public BasicOpmode(double driveLoopIterations){
        this.driveLoopIterations = driveLoopIterations;
        debug = false;
    }

    public BasicOpmode(double driveLoopIterations, boolean debug){
        this.driveLoopIterations = driveLoopIterations;
        this.debug = debug;
    }

    @Override
    public void runOpMode() {
        statemachine = new StateMachine();
        robot = new Hardware(this, telemetry);
        stateMachineSwitcher = new StateMachineSwitcher();
        mixer = new SoundMixer(hardwareMap.appContext);
        if(debug) {
            try {
                Connector.getInstance().start();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        setup();
        HardwareData hardware = new HardwareData(System.currentTimeMillis());
        robot.init();
        robot.calibrate();
        robotThread = new Thread(robot);
        robotThread.start();
        double currentLoops = 1;
        while(!isStopRequested()){
            hardware.setTimestamp(System.currentTimeMillis());
            robot.setData(hardware);
            SensorData sensors = robot.getSensors();
            if(!stateMachineSwitcher.isStarted()){
                stateMachineSwitcher.start(sensors, hardware);
            }
            telemetry.addData("Hardware Latency", System.currentTimeMillis() - hardware.getTimestamp());
            telemetry.addData("Sensors Latency", System.currentTimeMillis() - sensors.getTimestamp());
            telemetry.addData("Active Manager", stateMachineSwitcher.getActiveManager());
            if(debug){
                Connector.getInstance().addTelemetry("Hardware Latency", System.currentTimeMillis() - hardware.getTimestamp());
                Connector.getInstance().addTelemetry("Sensors Latency", System.currentTimeMillis() - sensors.getTimestamp());
                Connector.getInstance().addTelemetry("Active Manager", stateMachineSwitcher.getActiveManager());
                //Connector.getInstance().addSensorIO("Gyro", robot.getAllGyroAngles().toString());
            }
            statemachine.update(sensors, hardware);
            currentLoops -= (1/driveLoopIterations);
            if(currentLoops <= 0){
                hardware.setMotorPowers(statemachine.getDriveVelocities(sensors));
                currentLoops = 1;
            }
            stateMachineSwitcher.update(sensors, hardware);
            telemetry.update();
            if(debug) {
                try {
                    if(System.currentTimeMillis() > timer) {
                        Connector.getInstance().update();
                        timer = System.currentTimeMillis() + 100;
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (JSONException e) {
                    e.printStackTrace();
                }
            }
        }
        robotThread.interrupt();
        mixer.stopAll();
    }

    public static void setRedMovements(HashMap<String, Vector3> movements, HashMap<String, Vector3> turns){
        for(String s : movements.keySet()){
            movements.get(s).setA(movements.get(s).getA() * -1);
            movements.get(s).setC(360 - movements.get(s).getC());
        }
        for(String s : turns.keySet()){
            turns.get(s).setC(360 - turns.get(s).getC());
        }
    }

    public abstract void setup();
}
