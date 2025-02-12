package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    enum CLAW {
        IDLE,
        DROP,
        GRAB
    }
    CLAW state = CLAW.IDLE;
    ElapsedTime clawTimer = new ElapsedTime();

    enum SPECPIVOT {
        IDLE,
        SPEC,
        INTAKE
    }
    SPECPIVOT specState = SPECPIVOT.IDLE;
    ElapsedTime specTimer = new ElapsedTime();

    enum PIVOT {
        IDLE,
        SPEC,
        INTAKE
    }
    PIVOT pivotState = PIVOT.IDLE;
    ElapsedTime pivotTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ActionScheduler scheduler = new ActionScheduler();
        Robot robot = new Robot(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        boolean isClawClosed = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setColorRed();

        robot.arm.intakePrimePosition();

        while (opModeInInit()) {

        }

        waitForStart();
        robot.intake.leftLight.setPosition(0);
        robot.intake.rightLight.setPosition(0);
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad2.touchpad) {
                if (Intake.selected_color.equalsIgnoreCase("R")) {
                    robot.intake.setColorBlue();
                    gamepad2.setLedColor(0, 0, 255, -1);
                } else {
                    robot.intake.setColorRed();
                    gamepad2.setLedColor(255, 0, 0, -1);
                }
            }
            telemetry.addData("Selected Color", Intake.selected_color);

            //Drive
            gamepad1.setLedColor(255, 255, 0, -1);
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            if (gamepad1.right_bumper) robot.arm.grabber.setPosition(Arm.OPEN);

            // Operator

            //Intake & Color detection

            if (!scheduler.isBusy()) {
                robot.intake.manualControl(-gamepad2.left_stick_y);
                robot.lift.manualControl(-gamepad2.right_stick_y);
            }

            if ((gamepad2.right_trigger > 0.1) && !scheduler.isBusy()) {
                robot.intake.spin.setPower(1);
                intake.intakeDown();
            } else if ((gamepad2.left_trigger > 0.1) && !scheduler.isBusy()) {
                scheduler.queueAction(robot.eject());
            } else {
                robot.intake.spin.setPower(0);
                robot.intake.intakeUp();
            }

            if (robot.intake.hasSample() && !robot.intake.isRightColor() && !scheduler.isBusy()) {
                scheduler.queueAction(robot.eject());
            }

            if (robot.intake.hasSample() && robot.intake.isRightColor() && !scheduler.isBusy()) {
                scheduler.queueAction(robot.transfer());
            }

            //Outtake
            if ((gamepad2.dpad_down) && !scheduler.isBusy()) {
                //specimen
                robot.outtakeSpec();
            }

            if ((gamepad2.dpad_right) && !scheduler.isBusy()) {
                //observation drop
                robot.outtakeObservation();
            }

            if ((gamepad2.dpad_left) && !scheduler.isBusy()) {
                //low bucket
                robot.outtakeLowBucket();
            }

            if ((gamepad2.dpad_up) && !scheduler.isBusy()) {
                //high bucket
                robot.outtakeBucket();
            }
//            if ((gamepad2.left_bumper) && !scheduler.isBusy()) {
//                //at some point FSM that moves the arm out the first press and grabs & primes drop the second press
//                robot.specIntake();
//            }

//            if ((gamepad2.right_bumper) && !scheduler.isBusy()) {
//                //drop & return
//                if (arm.wristPosition == arm.WRIST_BUCKET_PRIME) {
//                    scheduler.queueAction(robot.sampleDropAndReturn());
//                } else {
//                    scheduler.queueAction(robot.specDropAndReturn());
//                }
//            }

            if (gamepad2.triangle) {
                robot.lift.resetEncoder();
                robot.intake.resetEncoder();
            }

            //Hang

            if ((gamepad2.square) && !scheduler.isBusy()) {
                scheduler.queueAction(robot.hang.hangIn());
            }

            if ((gamepad2.circle) && !scheduler.isBusy()) {
                scheduler.queueAction(robot.hang.hangOut());
            }

            switch (specState) {
                case IDLE:
                    specTimer.reset();
                    if (gamepad2.left_bumper) specState = SPECPIVOT.SPEC;
                    break;
                case SPEC:
                    arm.grab();
                    if (specTimer.seconds() > 0.3) {
                        specTimer.reset();
                        specState = SPECPIVOT.INTAKE;
                    }
                    break;
                case INTAKE:
                    arm.specIntake();
                    Lift.targetPosition = Lift.SPECIMEN_PICKUP;
                    arm.drop();
                    if (specTimer.seconds() > 0.5) specState = SPECPIVOT.IDLE;
                    break;
            }


            switch (pivotState) {
                case IDLE:
                    pivotTimer.reset();
                    if (gamepad2.right_bumper) pivotState = PIVOT.SPEC;
                    break;
                case SPEC:
                    if (arm.wristPosition == arm.WRIST_BUCKET_PRIME) {
                        scheduler.queueAction(robot.sampleDrop());
                    } else {
                        scheduler.queueAction(robot.specDrop());
                        arm.grab();
                        if (pivotTimer.seconds() > 0.3) {
                            pivotTimer.reset();
                            pivotState = PIVOT.INTAKE;
                        }
                    }
                    break;
                case INTAKE:
                    arm.intakePrimePosition();
                    Lift.targetPosition = Lift.SPECIMEN_PICKUP;
                    arm.clawPrime();
                    if (pivotTimer.seconds() > 0.5) pivotState = PIVOT.IDLE;
                    break;
            }


            switch (state) {
                case IDLE:
                    clawTimer.reset();
                    if (gamepad2.cross) state = CLAW.GRAB;
                    break;
                case GRAB:
                    robot.arm.grab();
                    if (gamepad2.cross && clawTimer.seconds() > 0.3) {
                        clawTimer.reset();
                        state = CLAW.DROP;
                    }
                    break;
                case DROP:
                    robot.arm.drop();
                    if (clawTimer.seconds() > 0.5) state = CLAW.IDLE;
                    break;
            }

            telemetry.update();
            scheduler.update();
            robot.update();
        }
        }
    }
