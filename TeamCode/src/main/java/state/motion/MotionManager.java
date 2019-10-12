package state.motion;

import drivetrain.MecanumDrive;
import math.Vector2;
import math.Vector3;
import state.StateMachine;

public class MotionManager {
    CorrectionVector correctionVector;
    TurnCorrectionVector turnCorrectionVector;
    Vector3 position;
    MecanumDrive mecanumDrive;
    StateMachine stateMachine;
    String turnName, correctionName;
    public MotionManager(StateMachine stateMachine, Vector3 position, MecanumDrive mecanumDrive, CorrectionVector correctionVector, TurnCorrectionVector turnCorrectionVector, String turnName, String correctionName) {
        this.position = position;
        this.mecanumDrive = mecanumDrive;
        this.stateMachine = stateMachine;
        this.correctionVector = correctionVector;
        this.turnCorrectionVector = turnCorrectionVector;
        this.turnName = turnName;
        this.correctionName = correctionName;
    }

    public void update() {
        if(turnCorrectionVector.finished()){
            stateMachine.setActiveDriveState(correctionName);
            if(correctionVector.finished()){
                correctionVector.deactivateDriveState();
            }
        }
    }

    public void setTarget(Vector2 target, CorrectionVector.MOVEMENT_TYPE movement_type, double turnTolerance, double kp, double moveTolerance){
        correctionVector.start(target, movement_type, position, new Vector2(position.getA(), position.getB()).angleTo(target), kp, moveTolerance);
        turnCorrectionVector.start(kp, new Vector2(position.getA(), position.getB()).angleTo(target), turnTolerance, position);
        stateMachine.setActiveDriveState(turnName);
    }
}
