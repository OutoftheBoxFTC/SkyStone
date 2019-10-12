package hardware.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class SmartGamepad {
    private Gamepad gamepad;
    public Button a, b, x, y, rightBumper, leftBumper, dpadDown, dpadUp, dpadLeft, dpadRight;

    public double rightStickX, rightStickY, leftStickX, leftStickY, rightTrigger, leftTrigger;

    public SmartGamepad(Gamepad gamepad){
        this.gamepad = gamepad;
        a = new Button();
        b = new Button();
        x = new Button();
        y = new Button();
        rightBumper = new Button();
        leftBumper = new Button();
        dpadDown = new Button();
        dpadLeft = new Button();
        dpadRight = new Button();
        dpadUp = new Button();
    }

    public void update(){
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        rightBumper.update(gamepad.right_bumper);
        leftBumper.update(gamepad.left_bumper);
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        rightStickX = gamepad.right_stick_x;
        rightStickY = gamepad.right_stick_y;
        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightTrigger = gamepad.right_trigger;
        leftTrigger = gamepad.left_trigger;
    }


}