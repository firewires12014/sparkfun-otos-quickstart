package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Testing OpMode", group="Into the Deep")
public class Testing extends LinearOpMode {
    public DcMotorEx spin;
    @Override
    public void runOpMode() throws InterruptedException {
        spin = hardwareMap.get(DcMotorEx.class, "spin");
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Game Pad 1 Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Game Pad 1 Right Trigger", gamepad1.right_trigger);
            telemetry.update();

            spin.setPower(gamepad1.left_trigger);
            spin.setPower(-gamepad1.right_trigger);
        }
    }
}
