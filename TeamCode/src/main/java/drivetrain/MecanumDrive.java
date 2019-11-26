package drivetrain;

import math.Matrix34;
import math.Matrix43;
import math.Vector3;
import math.Vector4;

public class MecanumDrive extends RobotDrive {

    private Matrix43 velocityTransformation;
    private Matrix34 wheelTransformation;
    public MecanumDrive(Polarity p, double theta, double rotationFactor){
        int polarity = p.getP();
        double l = 4.0*polarity/ Math.sin(theta),
                m = 4.0,
                n = 4.0/rotationFactor;
        velocityTransformation = new Matrix43(
                -l, m, -n,
                 l, m,  n,
                 l, m, -n,
                -l, m,  n
        ).scale(0.25);

        wheelTransformation = new Matrix34(
                -1/l, 1/l, 1/l, -1/l,
                1/m, 1/m, 1/m, 1/m,
                -1/n, 1/n, -1/n, 1/n
        );
    }

    @Override
    public Vector4 getWheelVelocities(Vector3 velocity) {
        velocity = new Vector3(-velocity.getB(), velocity.getA(), velocity.getC());
        return velocityTransformation.transform(velocity);
    }

    @Override
    public Vector3 getRobotVelocity(Vector4 wheels) {
        Vector3 velocity = wheelTransformation.transform(wheels);
        return new Vector3(velocity.getB(), -velocity.getB(), velocity.getC());
    }

    public enum Polarity{
        IN(-1),
        OUT(1);
        private final int p;
        Polarity(int p) {
            this.p = p;
        }

        public int getP() {
            return p;
        }
    }
}
