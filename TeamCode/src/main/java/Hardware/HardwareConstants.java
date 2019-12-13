package Hardware;

import math.Vector2;

/**
 * Constants for hardware devices
 */
public class HardwareConstants {
    //Servos
    public static final Vector2 LATCH_ON = new Vector2(0.25, 0.95);
    public static final Vector2 LATCH_OFF = new Vector2(0.95, 0.29);
    public static final Vector2 OPEN_INTAKE = new Vector2(0.95, 1-0.95);
    public static final Vector2 CLOSE_INTAKE = new Vector2(0.6, 1-0.6);
    public static final Vector2 LOCKED_INTAKE = new Vector2(0.45, 1-0.45);
    public static final double INTAKE_LATCH_ON = 0.85;
    public static final double INTAKE_LATCH_OFF = 0.3678;
    public static final double LIFT_REST = 0.015;
    public static final double LIFT_OUT = 0.9;
    public static final double LIFT_REST_OFFSET = -0.0069;
    public static final double LIFT_OUT_OFFSET = 0.00939;
}
