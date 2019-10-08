package state;

import Odometer.Odometer;
import hardware.ReadData;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import Odometer.SimpleDynamicIncrements;

public class Orientation extends LogicState {
    private static final double TAU = Math.PI*2;

    private Vector3 position, velocity;
    private Odometer odometer;

    private double initialRotation;

    public Orientation(StateMachine stateMachine, Odometer odometer, Vector3 position, Vector3 velocity) {
        super(stateMachine);
        this.odometer = odometer;
        this.position = position;
        this.velocity = velocity;
        initialRotation = position.getC();
    }

    @Override
    public void init(ReadData data) {
        odometer.calibrate(data);
    }

    @Override
    public void update(ReadData data) {
        velocity.set(odometer.getVelocity(data));

        Matrix22 rotation = rotationMatrix(odometer.getGlobalDynamics().getC());
        SimpleDynamicIncrements dynamicRobotIncrements = odometer.updateRobotDynamics(data);
        Vector3 globalDynamics = odometer.getGlobalDynamics();

        double newRotation = ((initialRotation+globalDynamics.getC())%TAU+TAU)%TAU;//limits rotation to between 0 (inclusive) and 2pi (exclusive)




        Vector2 staticRobotIncrements = odometer.findStaticIncrements(dynamicRobotIncrements);
        Vector2 fieldIncrements = rotation.transform(staticRobotIncrements);
        position.set(position.add(new Vector3(fieldIncrements.getA(), fieldIncrements.getB(), 0)));
        position.setC(Math.toDegrees(data.getGyro()));
    }

    private Matrix22 rotationMatrix(double rotation){
        double cos = Math.cos(rotation),
                sine = Math.sin(rotation);
        return new Matrix22(cos, -sine, sine, cos);
    }
}
