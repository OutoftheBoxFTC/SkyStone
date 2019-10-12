package odometry;

import hardware.ReadData;
import math.Vector2;
import math.Vector3;

public abstract class Odometer {
    //TODO tune me
    private static final double
            ROTATION_FACTOR = 1.398856E-4,
            TRANSLATION_FACTOR = 0.0010329132,
            AUX_ROTATION_FACTOR = .467625745;

    protected double rotationFactor, translationFactor, auxRotationFactor;
    public Odometer(double rotationFactor, double translationFactor, double auxRotationFactor){
        this.rotationFactor = rotationFactor;
        this.translationFactor = translationFactor;
        this.auxRotationFactor = auxRotationFactor;
    }

    public Odometer(){
        this.rotationFactor = ROTATION_FACTOR;
        this.translationFactor = TRANSLATION_FACTOR;
        this.auxRotationFactor = AUX_ROTATION_FACTOR;
    }

    public abstract SimpleDynamicIncrements updateRobotDynamics(ReadData data);

    public abstract Vector2 findStaticIncrements(SimpleDynamicIncrements data);
    public abstract Vector3 getGlobalDynamics();

    public void setFactors(double rotation, double translation, double auxRotation){
        rotationFactor = rotation;
        translationFactor = translation;
        auxRotationFactor = auxRotation;
    }

    public Vector3 getVelocity(ReadData data){
        double left = data.getvLeft(), right = data.getvRight(), aux = data.getvAux();
        double rotation = right-left,
                fwd = (right+left)/2,
                strafe = aux-rotation*auxRotationFactor;
        return new Vector3(strafe*translationFactor, fwd*translationFactor, rotation*rotationFactor);
    }

    /**
     * updates and offsets global dynamics so that the next static increment don't attempt to compensate for distance traveled between previous and current update
     */
    public abstract void calibrate(ReadData data);
}
