package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.FArm;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    enum ROBOT_STATE {
        SCORE_HIGH_BUCKET,
        SCORE_LOW_BUCKET,
    }

    public enum INTAKE_STATE {
        IN, // Pivot up, extension full retracted
        OUT, // Pivot horizontal, out slightly
        INTAKING, // Pivot down, extension doesn't matter
        OUTTAKING, // Pivot horizontal, extension doesn't matter
        TRANSFER
    }

    public enum FARM_STATE {
        SPEC_INTAKE,
        LOW_BUCKET_SCORE,
        TRANSFER,
        SPEC_SCORE,
        BUCKET_SCORE,
        RETURN_LIFT
    }

    public enum TRANSFER_STATE {
        RETURNING,
        WAIT,
        GRAB,
        IDLE
    }

    FARM_STATE farmState = FARM_STATE.TRANSFER;
    INTAKE_STATE intakeState = INTAKE_STATE.IN;
    TRANSFER_STATE transferState = TRANSFER_STATE.IDLE;

    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime clawTimer = new ElapsedTime();

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0,0), LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        robot.farm.setTransfer();
        prevLoop = System.nanoTime() / 1e9;
        transferTimer.reset();
        while (opModeIsActive() && !isStopRequested()) {
            robot.clearBulkCache(); // Do once per loop
            // Calum
            // Drivetrain
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            // Wasi
            // Lift
            robot.farm.manualControl(-gamepad2.right_stick_y);

            // Intake
            robot.intake.manualControl(-gamepad2.left_stick_y);

            if (gamepad2.right_trigger > 0.1) {
                intakeState = INTAKE_STATE.INTAKING;
            } else if(gamepad2.left_trigger > 0.1) {
                intakeState = INTAKE_STATE.OUTTAKING;
            } else {
                if (transferState.equals(TRANSFER_STATE.IDLE)) {
                    if (robot.intake.isOuttakeOut()) intakeState = INTAKE_STATE.OUT;
                    else intakeState = INTAKE_STATE.IN;
                }
            }

            // Still figure out transfer
            // assuming it transfers (grabs block that's it) then when intake is in manual open claw
            if (!Intake.PID_ENABLED && !robot.intake.holdIn) robot.farm.drop(); // make add a condition for spec (ion know when it would ever be the case)

            if (intakeState.equals(INTAKE_STATE.INTAKING) && robot.hasSample()) {
                intakeState = INTAKE_STATE.TRANSFER;
                farmState = FARM_STATE.TRANSFER;
                transferState = TRANSFER_STATE.RETURNING;
            }

            if (gamepad2.dpad_down) farmState = FARM_STATE.SPEC_INTAKE;
            if (gamepad2.dpad_left) farmState = FARM_STATE.LOW_BUCKET_SCORE;
            if (gamepad2.dpad_up && transferState.equals(TRANSFER_STATE.IDLE)) farmState = FARM_STATE.BUCKET_SCORE;
            if (gamepad2.dpad_right) farmState = FARM_STATE.SPEC_SCORE;

            if (gamepad2.right_bumper) {
                robot.farm.setTransfer();
                FArm.targetPosition = 0;
                farmState = FARM_STATE.RETURN_LIFT;
            }

            if (gamepad2.cross && clawTimer.seconds() > 0.3) {
                if (robot.farm.isClawOpen()) robot.farm.close();
                else robot.farm.drop();
                clawTimer.reset();
            }

//            if (gamepad2.square && robot.inRangeOfBucket()) robot.farm.drop(); // auto drop

            if (gamepad2.triangle) {
                robot.intake.resetEncoder();
                robot.farm.resetEncoder();
            }

            switch (farmState) {
                case SPEC_INTAKE:
                    robot.farm.setSpecIntake();
                    break;
                case TRANSFER:
                    // Is in state
                    break;
                case SPEC_SCORE:
                    robot.farm.setSpecScore();
                    break;
                case BUCKET_SCORE:
                    robot.farm.setBucketScore();
                    break;
                case LOW_BUCKET_SCORE:
                    robot.farm.setBucketScore(true);
                    break;
                case RETURN_LIFT:
                    robot.turnOffLight();
                    FArm.PID_ENABLED = false;
                    robot.farm.lift.setPower(-1);
                    robot.farm.lift2.setPower(-1);

                    if (robot.farm.lift.getCurrent(CurrentUnit.MILLIAMPS) > 7000 || robot.farm.lift2.getCurrent(CurrentUnit.MILLIAMPS) > 7000) {
                        robot.farm.resetEncoder();
                        FArm.targetPosition = 0;
                        FArm.PID_ENABLED = true;

                        farmState = FARM_STATE.TRANSFER;
                    }
            }

            switch (intakeState) {
                case IN:
                    robot.intake.retractIntake();
                    robot.intake.intakeUp();
                    robot.intake.stopIntake();
                    if (robot.intake.extension.getCurrentPosition() < 20) {
                        robot.intake.stopIntake();
                        robot.intake.holdIn = true;
                    } else {
                        robot.intake.startIntake();
                        robot.intake.holdIn = false;
                    }
                    //robot.farm.close();
                    break;
                case OUT: // Sorta idle
                    robot.intake.intakeHorizontal();
                    robot.intake.stopIntake();
                    robot.farm.drop();
                    robot.intake.holdIn = false;
                    break;
                case INTAKING:
                    robot.intake.intakeDown();
                    robot.intake.startIntake();
                    robot.intake.holdIn = false;
                    break;
                case OUTTAKING:
                    robot.intake.intakeHorizontal();
                    robot.intake.reverseIntake();
                    robot.intake.holdIn = false;
                    break;
                case TRANSFER:
                    switch (transferState) {
                        case RETURNING:
                            Intake.PID_ENABLED = true;
                            robot.farm.setTransfer();
                            robot.intake.intakeUp();
                            Intake.PID_ENABLED = false;
                            robot.intake.extension.setPower(-1);
//                            robot.intake.retractIntake();

                            if (robot.intake.extension.getCurrentPosition() < 50) {
                                transferTimer.reset();
                                transferState = TRANSFER_STATE.WAIT;
                            }
                            break;
                        case WAIT:
                            robot.intake.holdIn = true;

                            if (transferTimer.seconds() > 0.2) {
                                robot.farm.close();
                                transferTimer.reset();
                                transferState = TRANSFER_STATE.GRAB;
                            }
                            break;
                        case GRAB:
                            Intake.PID_ENABLED = true;
                            robot.farm.close();

                            if (transferTimer.seconds() > 0.2) {
                                transferState = TRANSFER_STATE.IDLE;
                            }
                            break;
                        case IDLE:
                            break;
                    }
                default:
                    // ERROR
                    break;
            }

            robot.update();
            telemetry.addLine("---Intake---");
            telemetry.addData("State", intakeState);
            telemetry.addLine("---FArm---");
            telemetry.addData("State", farmState);
            telemetry.addData("Transfer State", transferState);
            telemetry.addData("Sample Color Red", robot.colorValueRed);
            telemetry.addData("Sample Color Blue", robot.colorValueBlue);
            telemetry.addData("Sample Color Green", robot.colorValueGreen);
            loopTimeMeasurement(telemetry); // replaces telemetry.update()
        }
    }

    public void loopTimeMeasurement(Telemetry telemetry) {
        double currTime = System.nanoTime() / 1e9;
        double delta = currTime - prevLoop;
        telemetry.addData("loop time", delta / 1000);
        telemetry.addData("hz", 1 / delta);
        telemetry.update();
        prevLoop = System.nanoTime() / 1e9;
    }
}
