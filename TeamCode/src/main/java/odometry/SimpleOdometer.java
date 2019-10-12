package odometry;

import hardware.ReadData;
import math.Matrix33;
import math.Vector2;
import math.Vector3;

public class SimpleOdometer extends Odometer {
    private Vector3 globalRobotDynamics, globalDynamicsOffset;

    public SimpleOdometer(double rotationFactor, double translationFactor, double auxRotationFactor){
        super(rotationFactor, translationFactor, auxRotationFactor);
        globalRobotDynamics = new Vector3(0, 0, 0);
        globalDynamicsOffset = new Vector3(0, 0, 0);
    }

    public SimpleOdometer(){
        super();
        globalRobotDynamics = new Vector3(0, 0, 0);
        globalDynamicsOffset = new Vector3(0, 0, 0);
    }

    public SimpleDynamicIncrements updateRobotDynamics(ReadData data){
        int left = data.getLeft(), right = data.getRight(), aux = data.getAux();

        double newRotation = (right-left)/2;
        double newFwd = (left+right)/2;
        double newStrafe = aux-newRotation*auxRotationFactor;

        double rotationIncrement = (newRotation-globalRobotDynamics.getC())*rotationFactor;
        double fwdIncrement = (newFwd-globalRobotDynamics.getB())*translationFactor;
        double strafeIncrement = (newStrafe-globalRobotDynamics.getA())*translationFactor;

        globalRobotDynamics.set(newStrafe, newFwd, newRotation);
        return new SimpleDynamicIncrements(new Vector3(strafeIncrement, fwdIncrement, rotationIncrement));
    }

    public Vector2 findStaticIncrements(SimpleDynamicIncrements dynamics){
        Vector3 robotIncrements = dynamics.getDynamicRobotIncrements();

        double strafe = robotIncrements.getA(),
                fwd = robotIncrements.getB(),
                rot = robotIncrements.getC();
        double cos = Math.cos(rot), sine = Math.sin(rot);

        double y = (fwd*sine + strafe*(1-cos))/rot;
        double x = (strafe*sine - fwd*(1-cos))/rot;
        if(rot==0){
            y=fwd;
            x=strafe;
        }
        return new Vector2(x, y);
    }

    @Override
    public Vector3 getGlobalDynamics() {
        return new Matrix33(globalRobotDynamics.subtract(globalDynamicsOffset)).transform(new Vector3(translationFactor, translationFactor, rotationFactor));
    }

    @Override
    public void calibrate(ReadData data) {
        Vector3 oldDynamics = globalRobotDynamics.clone();
        updateRobotDynamics(data);
        Vector3 newDynamics = globalRobotDynamics.clone();
        globalDynamicsOffset.set(newDynamics.subtract(oldDynamics));
    }
}