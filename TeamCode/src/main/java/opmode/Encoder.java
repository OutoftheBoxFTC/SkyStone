package opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name = "encoder")
public class Encoder extends LinearOpMode {
    @Override
    public void runOpMode() {
        ExpansionHubEx hub = hardwareMap.get(ExpansionHubEx.class, "hub1");
        DcMotor a = hardwareMap.dcMotor.get("1"), b=hardwareMap.dcMotor.get("2"), c=hardwareMap.dcMotor.get("3"), d=hardwareMap.dcMotor.get("4");
        waitForStart();
        while (opModeIsActive()){
            RevBulkData data = hub.getBulkInputData();
            telemetry.addData("0", a.getCurrentPosition());
            telemetry.addData("1", b.getCurrentPosition());
            telemetry.addData("2", c.getCurrentPosition());
            telemetry.addData("3", d.getCurrentPosition());
            telemetry.update();
        }
    }
}
