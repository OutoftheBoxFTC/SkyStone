package HardwareSystems;

import State.LogicState;
import State.StateMachine;
import math.Vector2;

/**
 * Constants for hardware devices
 * All direction are assuming you are behind the robot (on the side where we outtake the block)
 */
public class HardwareConstants {
    //Servos
    public static final Vector2 LATCH_ON = new Vector2(0.2, 0.95); //The servo positions for latching ON the foundation. It is in the order {left, right}
    public static final Vector2 LATCH_OFF = new Vector2(0.70, 0.29); //The servo positions for latching OFF the foundation. It is in the order {left, right}
    public static final Vector2 OPEN_INTAKE = new Vector2(0.95, 1-0.95); //The servo positions for OPENING the intake. It is in the order {left, right}
    public static final Vector2 CLOSE_INTAKE = new Vector2(0.50, 1-0.55); //The servo positions for CLOSING the intake. It is in the order {left, right}
    public static final Vector2 CLOSE_INTAKE_TELEOP = new Vector2(0.65, 1-0.70); //The servo positions for CLOSING the intake IN TELEOP. It is in the order {left, right}
    public static final double INTAKE_LATCH_ON = 0.65; //The servo position for the latch on the four bar lift to latch ON to the block
    public static final double INTAKE_LATCH_OFF = 0.1; //The servo position for the latch on the four bar lift to latch OFF of the block
    public static final Vector2 LIFT_REST = new Vector2(SQUARE_WAVE_TO_POSITION(780), SQUARE_WAVE_TO_POSITION(860)); //The servo position for the four bar servos when in the REST position, or all the way forward. It is in the order {left, right}
    public static final Vector2 LIFT_INTAKE = new Vector2(SQUARE_WAVE_TO_POSITION(850), SQUARE_WAVE_TO_POSITION(930)); //The servo position for the four bar servos when in the INTAKE position, or the position when intaking. It is in the order {
    public static final Vector2 LIFT_SCORING_POSITION = new Vector2(SQUARE_WAVE_TO_POSITION(1680), SQUARE_WAVE_TO_POSITION(1770)); //The servo position for the four bar servos when in the SCORING position. It is in the order {left, right}
    public static final Vector2 LIFT_OUT = new Vector2(SQUARE_WAVE_TO_POSITION(2000), SQUARE_WAVE_TO_POSITION(2015)); //The servo position for the four bar servos when in the OUT position, or when the arm is all the way out on the outtake side. It is in the order {left, right}
    public static final double CAPSTONE_LATCH_ON = SQUARE_WAVE_TO_POSITION(1800);
    public static final double CAPSTONE_LATCH_OFF = SQUARE_WAVE_TO_POSITION(900);
    public static LogicState resetLift(StateMachine stateMachine){
        return new LogicState(stateMachine) {
            int state = 0;
            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                if(state == 0) {
                    if (sensors.getLiftLimit()) {
                        hardware.setLiftMotors(-1);
                    }else{
                        state = 1;
                    }
                }else if(state == 1){
                    if (sensors.getLiftLimit()) {
                        state = 2;
                    }else{
                        hardware.setLiftMotors(1);
                    }
                }else if(state == 2){
                    if (sensors.getLiftLimit()) {
                        hardware.setLiftMotors(-0.5);
                    }else{
                        state = 3;
                    }
                }else if(state == 3){
                    if (sensors.getLiftLimit()) {
                        state = 4;
                    }else{
                        hardware.setLiftMotors(0.2);
                    }
                }else if(state == 4){
                    hardware.setLiftMotors(0);
                    deactivateThis();
                }
            }
        };
    }
    public static LogicState resetLift(StateMachine stateMachine, final String nextState){
        return new LogicState(stateMachine) {
            int state = 0;
            @Override
            public void update(SensorData sensors, HardwareData hardware) {
                if(state == 0) {
                    if (sensors.getLiftLimit()) {
                        hardware.setLiftMotors(0.7);
                    }else{
                        state = 1;
                    }
                }else if(state == 1){
                    if (sensors.getLiftLimit()) {
                        state = 2;
                    }else{
                        hardware.setLiftMotors(-0.4);
                    }
                }else if(state == 2){
                    if (sensors.getLiftLimit()) {
                        hardware.setLiftMotors(0.5);
                    }else{
                        state = 3;
                    }
                }else if(state == 3){
                    if (sensors.getLiftLimit()) {
                        state = 4;
                    }else{
                        hardware.setLiftMotors(-0.2);
                    }
                }else if(state == 4){
                    hardware.setLiftMotors(0);
                    stateMachine.activateLogic(nextState);
                    state = 0;
                    deactivateThis();
                }
            }
        };
    }
    public static double SQUARE_WAVE_TO_POSITION(double wave){
        //Converts a square wave value (500 to 2500) to a servo position (0 to 1)
        return (wave-500)/2000;
    }
}
