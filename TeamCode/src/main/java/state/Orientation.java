package state;

import hardware.ReadData;
import math.MathUtil;
import math.Matrix22;
import math.Vector2;
import math.Vector3;
import odometry.Odometer;
import odometry.SimpleDynamicIncrements;

public class Orientation extends LogicState {
    private static final double TAU = Math.PI*2;
    private static final long TIME_ADV = 6000000;

    private Vector3 actingPosition, truePosition, auxPosition;
    private static final Vector2 CENTER_OFFSET = new Vector2(0, 0);
    protected Odometer odometer;
    private long lastTime;
    private double initialRotation;

    public Orientation(StateMachine stateMachine, Odometer odometer, Vector3 actingPosition, Vector3 truePosition){
        super(stateMachine);
        this.odometer = odometer;

        this.actingPosition = actingPosition;
        this.truePosition = truePosition;
        actingPosition.set(truePosition);
        auxPosition = new Vector3(new Vector2(truePosition).add(MathUtil.rotationMatrix(truePosition.getC()).transform(CENTER_OFFSET).scale(-1)), truePosition.getC());

        initialRotation = actingPosition.getC();
        lastTime = 0;
    }

    public Orientation(StateMachine stateMachine, Odometer odometer, Vector3 truePosition){
        this(stateMachine, odometer, Vector3.ZERO(), truePosition);
    }

    @Override
    public void init(ReadData data) {
        lastTime = data.getDriveUpdateTime();
        odometer.calibrate(data);
    }

    @Override
    public void update(ReadData data) {
        Matrix22 rotation = MathUtil.rotationMatrix(odometer.getGlobalDynamics().getC());
        SimpleDynamicIncrements dynamicRobotIncrements = odometer.updateRobotDynamics(data);
        Vector3 globalDynamics = odometer.getGlobalDynamics();
        double newRotation = ((initialRotation+globalDynamics.getC())%TAU+TAU)%TAU;//limits rotation to between 0 (inclusive) and 2pi (exclusive)
        Vector2 staticRobotIncrements = odometer.findStaticIncrements(dynamicRobotIncrements);
        Vector2 fieldIncrements = rotation.transform(staticRobotIncrements);
        auxPosition.set(auxPosition.add(new Vector3(fieldIncrements.getA(), fieldIncrements.getB(), 0)));
        auxPosition.setC(newRotation);

        truePosition.set(auxPosition);
        truePosition.add(new Vector3(rotation.transform(CENTER_OFFSET), 0));

        actingPosition.set(truePosition);
        long dt = data.getDriveUpdateTime()-lastTime;
        lastTime = data.getDriveUpdateTime();
        SimpleDynamicIncrements interpolDynamicInc = new SimpleDynamicIncrements(dynamicRobotIncrements.getDynamicRobotIncrements().scale((double)TIME_ADV/dt));
        Vector2 interpolStaticInc = odometer.findStaticIncrements(interpolDynamicInc);
        Vector2 interpolFieldInc = MathUtil.rotationMatrix(odometer.getGlobalDynamics().getC()+interpolDynamicInc.getDynamicRobotIncrements().getC()).transform(interpolStaticInc);
        Vector3 interpol = new Vector3(interpolFieldInc, interpolDynamicInc.getDynamicRobotIncrements().getC());
        actingPosition.add(interpol);
    }
}
