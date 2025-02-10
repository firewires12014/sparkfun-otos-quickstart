package org.firstinspires.ftc.teamcode.opmode.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.subsystems.Hang;
import org.firstinspires.ftc.teamcode.subsystems.old.Intake;
import org.firstinspires.ftc.teamcode.subsystems.old.Lift;
import org.firstinspires.ftc.teamcode.subsystems.old.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.old.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

@Disabled
@TeleOp(name="Teleop", group="Into the Deep")
public class teleop extends LinearOpMode {
    boolean r2Toggle = false;

    enum DROP {
        IDLE,
        DROP,
        RETURN
    }

    DROP state = DROP.IDLE;

    boolean inInspect = false;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ActionScheduler scheduler = new ActionScheduler();
        Robot robot = new Robot(telemetry, hardwareMap);

        // Init
        robot.outtake.flipIn();

        waitForStart();
        timer.startTime();
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
//            } else if ((gamepad2.right_trigger > 0.1 && robot.intake.downSensor.getDistance(DistanceUnit.MM) < Intake.submerisbleBarDistance) || r2Toggle) { // doesn't work
//                r2Toggle = true;
//                robot.intake.down.setPosition(Intake.fourbarDown);
//                robot.intake.spin.setPower(1);
//                robot.intake.lock.setPosition(Intake.GEEKED);
            }  else if (gamepad2.left_trigger > 0.1) {
                  robot.intake.down.setPosition(Intake.fourbarDown);
                  robot.intake.spin.setPower(1);
            } else {
                r2Toggle = false;
                if (!scheduler.isBusy() && !inInspect) {
                    robot.intake.spin.setPower(0);
                    robot.intake.down.setPosition(Intake.fourbarResting);
                }
            }

            // Hang
            if (gamepad2.cross) {
//                scheduler.queueAction(robot.hang.hangIn());
            }
            if (gamepad2.triangle) {
//                scheduler.queueAction(robot.hang.hangOut());
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

            if (gamepad2.square) {
                robot.lift.resetEncoder();
                robot.intake.resetEncoder();
            }

            if (gamepad2.triangle) {
                robot.outtake.flipOut();
            }

            if (gamepad2.cross) {
                robot.outtake.flipIn();
            }

            if (gamepad2.circle) { // just release held object
                robot.outtake.drop();
            }

            if (gamepad2.dpad_down) {
                scheduler.queueAction(robot.depositOuttake());
            }

//            if (gamepad2.left_bumper && !scheduler.isBusy()) {
//                scheduler.queueAction(robot.dropAndReturnTeleop());
//            }

            // DROP FSM

            switch (state) {
                case IDLE:
                    timer.reset();
                    if (gamepad2.left_bumper) state = DROP.DROP;
                    break;
                case DROP:
                    robot.outtake.drop();
                    if (gamepad2.left_bumper && timer.seconds() > 0.3) {
                        timer.reset();
                        state = DROP.RETURN;
                    }
                    break;
                case RETURN:
                    robot.outtake.hold();
                    if (!scheduler.isBusy()) scheduler.queueAction(robot.dropAndReturnTeleop());
                    if (timer.seconds() > 0.5) state = DROP.IDLE;
                    break;
            }

            //



            if (gamepad2.dpad_left) {
                scheduler.queueAction(robot.outtakeSpecimen());
            }

            if (gamepad2.dpad_up) {
                scheduler.queueAction(robot.outtakeBucket());
            }

            if (gamepad2.right_bumper && !scheduler.isBusy()) {
                scheduler.queueAction(robot.transfer());
            }

            if (gamepad2.dpad_right) {
                scheduler.queueAction(robot.outtakeLowBucket());
            }

                robot.update();
                scheduler.update();

                telemetry.addData("LiftPower", robot.lift.lift.getPower());
                telemetry.addData("LiftPower", robot.lift.lift2.getPower());
                telemetry.addData("extensionDistance", robot.intake.extension.getCurrentPosition());
                telemetry.addData("distanceSensor", robot.intake.downSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("extensionTarget", Intake.targetPosition);
//                telemetry.addData("hangOutPosition", robot.hang.hangmotor1.getCurrentPosition());
//                telemetry.addData("hangInPosition", Hang.targetPosition);
                telemetry.addData("liftTarget", robot.lift.lift.getCurrentPosition());
                telemetry.addData("liftTarget", robot.lift.lift2.getCurrentPosition());
                telemetry.addData("Intake State", robot.intake.state);
                telemetry.addData("outtakeTargetPosition", Outtake.posCurrent);
                telemetry.addData("outtakeSetPosition", Outtake.targetPosition);
                telemetry.addData("liftSetPosition", Lift.targetPosition);
                telemetry.addData("Lift State", robot.lift.state);
                telemetry.addData("Lift PID", Lift.PID_ENABLED);
                telemetry.addData("Lift PID error", robot.lift.pid.getLastError());
                telemetry.addData("Lift new Power", robot.lift.newPower);
                telemetry.addData("Lift current", robot.lift.lift.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Lift current", robot.lift.lift2.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Schedule State", scheduler.isBusy());
                telemetry.addData("DROP FSM", state);
                telemetry.update();

            if (gamepad1.touchpad) {
                scheduler.queueAction(robot.intake.setTargetPositionAction(800));
                scheduler.queueAction(robot.outtake.moveOuttakeOut());
                scheduler.queueAction(new InstantAction(robot.outtake::flipOut));
                scheduler.queueAction(new InstantAction(robot.intake::fourbarOut));

                robot.intake.down.setPosition(Intake.fourbarDown);

                inInspect = true;
            }

        }
    }
}
