package drivetrain;

import math.Matrix34;
import math.Matrix43;
import math.Vector3;
import math.Vector4;

public class HolonomicDrive extends RobotDrive {
    private Matrix43 velocityTransformation;
    private Matrix34 wheelTransformation;
    public HolonomicDrive(double rotationFactor){
        double m = 1/2,
                n = rotationFactor/4;
       wheelTransformation = new Matrix34(
               m, 0, -m, 0,
               0, -m, 0, m,
               n, n, n, n
       );

        velocityTransformation = new Matrix43(
                2/m, 0, 1/n,
                0, -2/m, 1/n,
                -2/m, 0, 1/n,
                0, 2/m, 1/n
        ).scale(1/4);
    }


    @Override
    public Vector4 getWheelVelocities(Vector3 velocity) {
        return velocityTransformation.transform(velocity);
    }

    @Override
    public Vector3 getRobotVelocity(Vector4 wheels) {
        return wheelTransformation.transform(wheels);
    }
}
