package drivetrain;

import math.Vector3;
import math.Vector4;

public abstract class RobotDrive {

    /**
     *
     * @param velocity in strafe, fwd, and rotation
     * @return velocities of motors a, b, c, d
     */
    public abstract Vector4 getWheelVelocities(Vector3 velocity);

    /**
     *
     * @param wheels velocities of motors a, b, c, d
     * @return velocity in strafe, fwd, and rotation
     */
    public abstract Vector3 getRobotVelocity(Vector4 wheels);


}
