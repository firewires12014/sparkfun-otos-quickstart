package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

@TeleOp(name="Teleop", group="Into the Deep")
public class teleop extends LinearOpMode {
    boolean r2Toggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ActionScheduler scheduler = new ActionScheduler();
        Robot robot = new Robot(telemetry, hardwareMap);

        // Init
        robot.outtake.flipIn();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            // Driver
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // Operator
            // Intake

              if (gamepad2.right_trigger > 0.1) {
                robot.intake.down.setPosition(Intake.fourbarDown);
                robot.intake.spin.setPower(-1);
                robot.intake.lock.setPosition(Intake.SOMETHING_IN_BETWEEN);
            } else if ((gamepad2.right_trigger > 0.1 && robot.intake.downSensor.getDistance(DistanceUnit.MM) < Intake.submerisbleBarDistance) || r2Toggle) { // doesn't work
                r2Toggle = true;
                robot.intake.down.setPosition(Intake.fourbarDown);
                robot.intake.spin.setPower(1);
                robot.intake.lock.setPosition(Intake.GEEKED);
            }  else if (gamepad2.left_trigger > 0.1) {
                  robot.intake.down.setPosition(Intake.fourbarDown);
                  robot.intake.spin.setPower(1);
              } else {
                r2Toggle = false;
                if (!scheduler.isBusy()) {
                    robot.intake.spin.setPower(0);
                    robot.intake.down.setPosition(Intake.fourbarResting);
                }
            }

            // Hang
            if (gamepad2.cross) {
                scheduler.queueAction(robot.hang.hangIn());
            }
            if (gamepad2.triangle) {
                scheduler.queueAction(robot.hang.hangOut());
            }

            // Manual Control
//            robot.hang.manualControl(-gamepad2.right_stick_y);
            robot.intake.manualControl(-gamepad2.left_stick_y);
            if (!scheduler.isBusy()) {
               robot.lift.manualControl(-gamepad2.right_stick_y);
            }

//            if (gamepad2.left_bumper) {
//                scheduler.queueAction(robot.outtake.moveOuttakeIn());
//            }
//
//            if (gamepad2.left_trigger > .1) {
//                scheduler.queueAction(robot.outtake.moveOuttakeOut());
//            }

            if (gamepad2.touchpad) robot.lift.resetEncoder();

            if (gamepad2.circle) {
                robot.outtake.flipOut();
            }

            if (gamepad2.square) {
                robot.outtake.flipIn();
            }

            if (gamepad2.dpad_left) {
                scheduler.queueAction(robot.depositOuttake());
            }
            if (gamepad2.left_bumper && !scheduler.isBusy()) {
                scheduler.queueAction(robot.dropAndReturn());
            }

            if (gamepad2.dpad_right) {
                scheduler.queueAction(robot.outtakeSpecimen());
            }

            if (gamepad2.dpad_up) {
                scheduler.queueAction(robot.outtakeBucket());
            }

            if (gamepad2.right_bumper && !scheduler.isBusy()) {
                scheduler.queueAction(robot.transfer());
            }

                robot.update();
                scheduler.update();

                telemetry.addData("LiftPower", robot.lift.lift.getPower());
                telemetry.addData("extensionDistance", robot.intake.extension.getCurrentPosition());
                telemetry.addData("distanceSensor", robot.intake.downSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("extensionTarget", Intake.targetPosition);
                telemetry.addData("hangOutPosition", robot.hang.hangmotor1.getCurrentPosition());
                telemetry.addData("hangInPosition", Hang.targetPosition);
                telemetry.addData("liftTarget", robot.lift.lift.getCurrentPosition());
                telemetry.addData("Intake State", robot.intake.state);
                telemetry.addData("outtakeTargetPosition", Outtake.posCurrent);
                telemetry.addData("outtakeSetPosition", Outtake.targetPosition);
                telemetry.addData("liftSetPosition", Lift.targetPosition);
                telemetry.addData("Lift State", robot.lift.state);
                telemetry.addData("Lift PID", Lift.PID_ENABLED);
                telemetry.addData("Lift PID error", robot.lift.pid.getLastError());
                telemetry.addData("Lift new POewr", robot.lift.newPower);
                telemetry.addData("Lift current", robot.lift.lift.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.update();

        }
    }
}
