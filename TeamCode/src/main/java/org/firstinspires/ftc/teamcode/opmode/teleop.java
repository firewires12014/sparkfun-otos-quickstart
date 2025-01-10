package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

@TeleOp
public class
teleop extends LinearOpMode {
    boolean r2Toggle = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ActionScheduler scheduler = new ActionScheduler();
        Robot robot = new Robot(hardwareMap);
        waitForStart();
        while (opModeIsActive() && ! isStopRequested()) {
            robot.hang.hangmotor1.setPower(-gamepad1.left_stick_y);
//            if (gamepad1.square) {
//                scheduler.queueAction(robot.hang.engagePto());
//            }
//            if (gamepad1.circle) {
//                scheduler.queueAction(robot.hang.disengagePto());
//            }
            scheduler.queueAction(
                    robot.intake.manualControl(gamepad2)
            );
            if (gamepad2.right_bumper) {
               robot.intake.down.setPosition(Intake.fourbarDown);
               robot.intake.spin.setPower(1);
            }

            else if ((gamepad2.right_trigger > 0.1 && robot.intake.downSensor.getDistance(DistanceUnit.MM) < Intake.submerisbleBarDistance)|| r2Toggle) {
                r2Toggle = true;
                robot.intake.down.setPosition(Intake.fourbarDown);
                robot.intake.spin.setPower(1);
            } else {
                r2Toggle = false;
                robot.intake.down.setPosition(Intake.fourbarUp);
                robot.intake.spin.setPower(0);
            }

            if (gamepad2.cross) {
                scheduler.queueAction(
                        robot.hang.hangIn()
                );
            }

            if (gamepad2.triangle) {
                scheduler.queueAction(
                        robot.hang.hangOut()
                );
            }




            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.right_stick_y, 0),0));
            robot.lift.lift.setPower(-gamepad1.right_stick_y);
            robot.update();
            scheduler.update();
            telemetry.addData("LiftPower", robot.lift.lift.getPower());
            telemetry.addData("extensionDistance", robot.intake.extension.getCurrentPosition());
            telemetry.addData("distanceSensor", robot.intake.downSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("extensionTarget", Intake.targetPosition);
            telemetry.addData("hangOutPostion", robot.hang.hangmotor1.getCurrentPosition());
            telemetry.addData("hangInPosition", Hang.targetPosition);
            telemetry.addData("liftTarget", robot.lift.lift.getCurrentPosition());
            telemetry.addData("liftPosition", Lift.liftPosition);
            telemetry.update();
        }

    }
}
