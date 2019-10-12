package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import drivetrain.MecanumDrive;
import math.Vector3;
import math.Vector4;

@TeleOp(name = "Matrix Unit Test")
public class MatrixUnitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(MecanumDrive.Polarity.IN, Math.PI/4, 1);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("identity", drive.getRobotVelocity(drive.getWheelVelocities(new Vector3(1, 2, 3))));
            telemetry.addData("identity back", drive.getWheelVelocities(drive.getRobotVelocity(new Vector4(1, 2, 3, 4))));
            telemetry.update();
        }
    }
}
