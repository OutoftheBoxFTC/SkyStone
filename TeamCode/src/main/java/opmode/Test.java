package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import drivetrain.MecanumDrive;
import math.Vector3;
import math.Vector4;

@TeleOp(name = "Encoders")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);

        DcMotor a = hardwareMap.dcMotor.get("a");
        DcMotor b = hardwareMap.dcMotor.get("b");
        DcMotor c = hardwareMap.dcMotor.get("c");
        DcMotor d = hardwareMap.dcMotor.get("d");
        b.setDirection(DcMotorSimple.Direction.REVERSE);
        d.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("A", a.getCurrentPosition());
            telemetry.addData("B", b.getCurrentPosition());
            telemetry.addData("C", c.getCurrentPosition());
            telemetry.addData("D", d.getCurrentPosition());
            telemetry.update();
            Vector4 power = drive.getWheelVelocities(new Vector3(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x));
            a.setPower(power.getA());
            b.setPower(power.getB());
            c.setPower(power.getC());
            d.setPower(power.getD());
        }

    }
}