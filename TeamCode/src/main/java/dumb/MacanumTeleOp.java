package dumb;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MecanumTeleOp", group = "YEET")
public class MacanumTeleOp extends OpMode {
    Mecanum mecanum = new Mecanum(this);
    double servoPos = 0.5;
    @Override
    public void init() {
        mecanum.init();
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger >= 0.1) {
            mecanum.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }else{
            mecanum.driveRaw(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}
