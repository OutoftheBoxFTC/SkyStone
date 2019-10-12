package odometry;

public class StrafeOffset {
    protected static final double FORWARD_CIRCLE_CIRCUMFERENCE = 0,
        STRAFE_CIRCLE_CIRCUMFERENCE = 0;

    public static double getRotOffset(double forwardRotation){
        return (forwardRotation * STRAFE_CIRCLE_CIRCUMFERENCE) / FORWARD_CIRCLE_CIRCUMFERENCE;
    }
}
