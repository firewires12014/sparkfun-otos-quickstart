package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {
    ElapsedTime clawTimer = new ElapsedTime();
    ElapsedTime flickerTimer = new ElapsedTime();
    ElapsedTime hangTimer = new ElapsedTime();
    ElapsedTime intakeColorTimer = new ElapsedTime();
    ElapsedTime sampleColorTimer = new ElapsedTime();

    enum SPECGRAB {
        IDLE,
        SPECFRONT,
        SPECBACK,
        SPECWAIT,
        SPECLIFT,
        SPECSIGMA,
    }

    SPECGRAB specGrabState = SPECGRAB.IDLE;
    ElapsedTime specGrabTimer = new ElapsedTime();
    boolean hasWrongColor = false;

    public static double bucketDistance = 220;

    @Override
    public void runOpMode() throws InterruptedException {
        ActionScheduler scheduler = new ActionScheduler();
        Robot robot = new Robot(telemetry, hardwareMap);
        boolean isClawClosed = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setColorRed();

        robot.arm.intakePrimePosition();

        robot.intake.leftLight.setPosition(0);
        robot.intake.rightLight.setPosition(0);

        waitForStart();
        double lastLoopTime = System.nanoTime() / 1e9;
        while (opModeIsActive() && !isStopRequested()) {
// turn off leds
            if (gamepad2.touchpad && intakeColorTimer.seconds() > .5) {
                if (Intake.selected_color.equalsIgnoreCase("R")) {
                    robot.intake.setColorBlue();
                    gamepad2.setLedColor(0, 0, 255, -1);
                    robot.intake.leftLight.setPosition(Intake.BLUE);
                    robot.intake.rightLight.setPosition(Intake.BLUE);
                } else {
                    robot.intake.setColorRed();
                    gamepad2.setLedColor(255, 0, 0, -1);
                    robot.intake.leftLight.setPosition(Intake.RED);
                    robot.intake.rightLight.setPosition(Intake.RED);
                }
                intakeColorTimer.reset();
            }

            if (gamepad1.touchpad && sampleColorTimer.seconds() > .5) {
                telemetry.addLine("Touchpad Pressed GP1");
                if (Intake.selected_color.equalsIgnoreCase("R")) {
                    Robot.isSample = true;
                    gamepad1.setLedColor(255, 255, 0, -1);
                } else {
                    Robot.isSample = false;
                    gamepad1.setLedColor(48, 213, 200, -1);
                }
                sampleColorTimer.reset();
            }

            telemetry.addData("Selected Color", Intake.selected_color);
            telemetry.addData("Status", "Initializing");


            //robot.clearCache();

            //telemetry.addData("armDistance:", robot.arm.armSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Current Draw:", robot.lift.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Hang Encoder", robot.hang.hang.getCurrentPosition());

            //intake.currentColor();

            // Choose color
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

            /**
             * Driver Section
             */
            // Mecanum Drive Code - Set the colors of the controllers
            gamepad1.setLedColor(255, 255, 0, -1);
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            /**
             * Co-Pilot Section
             */

            /**
             * Manual Control Section
             */
            robot.intake.manualControl(-gamepad2.left_stick_y);
            robot.lift.manualControl(-gamepad2.right_stick_y);

            if ((gamepad2.right_trigger > 0.61) && (!hasWrongColor)) {
                robot.intake.spin.setPower(1);
                robot.intake.intakeDown();
               if (gamepad2.right_trigger < 0.6) {
                   robot.intake.intakeUp();
               }
            } else if (gamepad2.left_trigger > 0.1) {
                robot.intake.spin.setPower(-1);
            } else {
                // Intake Up and Stop Spin
                robot.intake.spin.setPower(0);
                robot.intake.intakeUp();
            }

            if (!scheduler.isBusy()) {
                // Intake Control
                    // Intake Down and Spi

                // Specimen Outtake
                if ((gamepad2.dpad_down)) {
                    robot.outtakeSpecTeleop();
                }

                // Observation Drop
                if ((gamepad2.dpad_right)) {
                    robot.outtakeObservation();
                }

                // Low Bucket Drop
                if ((gamepad2.dpad_left)) {
                    robot.outtakeLowBucket();
                }

                // High Bucket Drop
                if ((gamepad2.dpad_up)) {
                    robot.outtakeBucket();
                }

                // Reset Encoders
                if (gamepad2.triangle) {
                    robot.lift.resetEncoder();
                    robot.intake.resetEncoder();
                }

            }

            /**
             * Automated Section
             */

//            }

            if (gamepad2.square && !scheduler.isBusy() && robot.arm.getBucketDistance() < bucketDistance) {
                robot.arm.wrist.setPosition(robot.arm.WRIST_BUCKET_DROP);
                robot.arm.wristPosition = robot.arm.WRIST_BUCKET_DROP;
                robot.arm.drop();
                robot.intake.clearLED();
            }

            if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1) {
                if (robot.intake.hasSample() &&
                 //if(
                        robot.intake.isRightColor() &&
                        gamepad2.left_trigger < 0.1 &&
                        !scheduler.isBusy()) {
                     //scheduler.queueAction(robot.transfer());
                } else if (robot.intake.hasSample() &&
                 //} else if (
                        !robot.intake.isRightColor() &&
                        !scheduler.isBusy()) {
                    // TODO Move back into robot? Gamepad is not accessible as it is right now
                    scheduler.queueAction(robot.eject());
                }
            }

            robot.transferFSM();

            // Toggle The arm between specimen score and specimen intake
            switch (specGrabState) {
                case IDLE:
                    specGrabTimer.reset();
                    if (gamepad2.left_bumper) {
                        if (robot.arm.wristPosition == robot.arm.WRIST_INTAKE ||
                                robot.arm.wristPosition == robot.arm.WRIST_MIDDLE ||
                                robot.arm.wristPosition == robot.arm.WRIST_SPECIMEN_GRAB
                        ) {
                            specGrabState = SPECGRAB.SPECLIFT;
                        }
                    }
                    break;
                case SPECLIFT:
                    robot.arm.grab();
                    Lift.targetPosition = Lift.SPECIMEN_PICKUP;
                    if (specGrabTimer.seconds() > 0.5) {
                        specGrabTimer.reset();
                        specGrabState = SPECGRAB.SPECFRONT;
                    }
                    break;
                case SPECFRONT:
                    scheduler.queueAction(
                            new SequentialAction(
                                    new InstantAction(robot.arm::autoSpecIntake),
                                    new InstantAction(() -> {
                                         robot.arm.drop();
                                    })
                            )
                    );
                    if (robot.lift.lift.getCurrentPosition() == Lift.SPECIMEN_DROP_PRIME) {
                        Lift.targetPosition = Lift.SPECIMEN_DROP;
                    }
                    specGrabTimer.reset();
                    specGrabState = SPECGRAB.SPECSIGMA;
                    break;
                case SPECSIGMA:
                    if (specGrabTimer.seconds() > .5) {
                        robot.arm.drop();
                        specGrabState = SPECGRAB.SPECBACK;
                    }
                    break;

                case SPECBACK:
                    if (specGrabTimer.seconds() > .5) {
                        specGrabTimer.reset();
                        Lift.targetPosition = Lift.SPECIMEN_PICKUP;
                        specGrabState = SPECGRAB.IDLE;
                    }
                    break;
            }

            if (gamepad2.right_bumper) {
                scheduler.queueAction(robot.returnIntake());
            }

            // Handle the grabber
            // If cross pressed and the grabber is closed, open it
            if (gamepad2.cross && clawTimer.seconds() > .5) {
                if (compareDouble(Arm.CLOSED, robot.arm.grabber.getPosition())) {
                    // If the wrist is in the bucket prime position, move it to the bucket drop position
                    if (Lift.targetPosition > 1700 || 450 < Lift.targetPosition && Lift.targetPosition < 600) {
                        robot.arm.setPivot(robot.arm.PIVOT_BUCKET);
                        robot.arm.wrist.setPosition(robot.arm.WRIST_BUCKET_DROP);
                        robot.arm.wristPosition = robot.arm.WRIST_BUCKET_DROP;
                    }
                    robot.arm.drop();
                    robot.intake.clearLED();
                    // If cross pressed and the grabber is open, close it
                } else if (compareDouble(Arm.OPEN, robot.arm.grabber.getPosition())) {
                    robot.arm.grab();
                }
                clawTimer.reset();
            }

            if (gamepad1.right_bumper && flickerTimer.seconds() > .5) {
                if (compareDouble(robot.intake.FLICKER_IN, robot.intake.flicker.getPosition())) {
                    robot.intake.flickerOut();
                } else if (compareDouble(robot.intake.FLICKER_OUT, robot.intake.flicker.getPosition())) {
                    robot.intake.flickerIn();
                }
                flickerTimer.reset();
            }

            if (gamepad2.right_stick_button) {
                scheduler.clearActions();
                robot.TRANSFER_STATE = Robot.tranfserState.IDLE;
            }

            if (gamepad2.circle && (flickerTimer.seconds() > .5)) {
                if (robot.hang.hang.getCurrentPosition() > 1500) scheduler.queueAction(robot.hang.hangIn());
                else scheduler.queueAction(robot.hang.hangOut());

                flickerTimer.reset();
            }

            if (gamepad2.right_stick_button) {
                scheduler.clearActions();
                robot.TRANSFER_STATE = Robot.tranfserState.IDLE;
            }

            scheduler.update();
            robot.update();

            double loopTimeMs = (System.nanoTime() / 1e9 - lastLoopTime);
            lastLoopTime = (System.nanoTime() / 1e9 - lastLoopTime);

            //telemetry.addData("Has Sample", robot.intake.hasSample());
            telemetry.addData("intake current", robot.intake.extension.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("Bucket D", robot.arm.getBucketDistance());
            telemetry.addData("Extension", robot.intake.extension.getCurrentPosition());
            telemetry.addData("Lift pos", robot.lift.lift.getCurrentPosition());
            telemetry.addData("Is Scheduler Busy", scheduler.isBusy());
            telemetry.addData("loop time (ms)", loopTimeMs);
            telemetry.addData("loop time (hz)", 1000/loopTimeMs);
            telemetry.update();
        }
    }

    public boolean compareDouble(double a, double b) {
        return Math.abs(a - b) < 0.0001;

    }
}
