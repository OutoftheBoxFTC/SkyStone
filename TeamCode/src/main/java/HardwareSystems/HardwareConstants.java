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
    public static final Vector2 LATCH_ON = new Vector2(0.10, 0.95); //The servo positions for latching ON the foundation. It is in the order {left, right}
    public static final Vector2 LATCH_OFF = new Vector2(0.70, 0.29); //The servo positions for latching OFF the foundation. It is in the order {left, right}
    public static final Vector2 OPEN_INTAKE = new Vector2(0.95, 1-0.95); //The servo positions for OPENING the intake. It is in the order {left, right}
    public static final Vector2 INTAKE_INIT = new Vector2(0.85, 1-0.85); //The servo positions for OPENING the intake. It is in the order {left, right}
    public static final Vector2 CLOSE_INTAKE = new Vector2(0.5, 1-0.5); //The servo positions for CLOSING the intake. It is in the order {left, right}
    public static final Vector2 CLOSE_INTAKE_AUTO = new Vector2(0.6, 1-0.6); //The servo positions for CLOSING the intake IN TELEOP. It is in the order {left, right}
    public static final double INTAKE_LATCH_ON = 0.65; //The servo position for the latch on the four bar lift to latch ON to the block
    public static final double INTAKE_LATCH_FEED = 0.25;
    public static final double INTAKE_LATCH_OFF = 0.05; //The servo position for the latch on the four bar lift to latch OFF of the block
    public static final Vector2 LIFT_REST = new Vector2(SQUARE_WAVE_TO_POSITION(820), SQUARE_WAVE_TO_POSITION(845)); //The servo position for the four bar servos when in the REST position, or all the way forward. It is in the order {left, right}
    public static final Vector2 LIFT_INTAKE = new Vector2(SQUARE_WAVE_TO_POSITION(840), SQUARE_WAVE_TO_POSITION(850)); //The servo position for the four bar servos when in the INTAKE position, or the position when intaking. It is in the order {
    public static final Vector2 LIFT_FEED_OUTTAKE = new Vector2(SQUARE_WAVE_TO_POSITION(940), SQUARE_WAVE_TO_POSITION(950));
    public static final Vector2 LIFT_SCORING_POSITION = new Vector2(SQUARE_WAVE_TO_POSITION(1785), SQUARE_WAVE_TO_POSITION(1770)); //The servo position for the four bar servos when in the SCORING position. It is in the order {left, right}
    public static final Vector2 LIFT_OUT = new Vector2(SQUARE_WAVE_TO_POSITION(2015), SQUARE_WAVE_TO_POSITION(2015)); //The servo position for the four bar servos when in the OUT position, or when the arm is all the way out on the outtake side. It is in the order {left, right}
    public static final Vector2 LIFT_OUT_AUTO = new Vector2(SQUARE_WAVE_TO_POSITION(1880), SQUARE_WAVE_TO_POSITION(1880));
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
