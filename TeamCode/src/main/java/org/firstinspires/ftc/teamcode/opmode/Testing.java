package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class Testing extends LinearOpMode {
    Robot robot;

    public static double leftArm = 0.0;
    public static double rightArm = 0.0;
    public static double grabber = 0.0;
    public static double wrist = 0.0;
    public static double inakePivot = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setColorRed();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
//            robot.arm.leftPivot.setPosition(leftArm);
//            robot.arm.rightPivot.setPosition(rightArm);
//            robot.arm.grabber.setPosition(grabber);
//            robot.arm.wrist.setPosition(wrist);
//            robot.intake.pivot.setPosition(inakePivot);

            robot.intake.manualControl(-gamepad2.left_stick_y);
            robot.lift.manualControl(-gamepad2.right_stick_y);

            if (gamepad2.square) robot.arm.setPivot(Arm.PIVOT_SPECIMEN_PICKUP);

            if (gamepad2.circle) robot.arm.setPivot(Arm.PIVOT_SPECIMEN_HORIZONTAL);

            robot.intake.spin.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            telemetry.addData("lift position", robot.lift.lift.getCurrentPosition());
            telemetry.addData("Lift target", Lift.targetPosition);
            telemetry.addData("Lift mode", Lift.PID_ENABLED);

            telemetry.addData("bucket dist", robot.arm.bucketSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Sub distance", robot.intake.subSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Sample Dist", robot.intake.sampleSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("sample RGB", robot.intake.currentColor());
            telemetry.addData("isWrongColor", robot.intake.isRightColor());
            telemetry.addData("hasSample", robot.intake.hasSample());
            robot.update();
            telemetry.update();
        }
    }
}
