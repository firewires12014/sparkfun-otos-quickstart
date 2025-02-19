package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;
import org.firstinspires.ftc.teamcode.util.ActionUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {
    ElapsedTime clawTimer = new ElapsedTime();

    enum SPECGRAB {
        IDLE,
        SPECFRONT,
        SPECBACK,
        SPECWAIT,
        SPECLIFT
    }

    SPECGRAB specGrabState = SPECGRAB.IDLE;
    ElapsedTime specGrabTimer = new ElapsedTime();
    boolean hasWrongColor = false;

    @Override
    public void runOpMode() throws InterruptedException {
        ActionScheduler scheduler = new ActionScheduler();
        Robot robot = new Robot(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        boolean isClawClosed = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setColorRed();

        robot.arm.intakePrimePosition();

        /**
         * Initialize the robot
         */
        while (opModeInInit()) {
            // turn off leds
            robot.intake.leftLight.setPosition(0);
            robot.intake.rightLight.setPosition(0);
            telemetry.addData("Status", "Initializing");
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("armDistance:", arm.armSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Current Draw:", lift.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            intake.currentColor();

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

            // Allow the driver to open the grabber/claw
            if (gamepad1.right_bumper) robot.arm.grabber.setPosition(Arm.OPEN);


            /**
             * Co-Pilot Section
             */

            /**
             * Manual Control Section
             */
            robot.intake.manualControl(-gamepad2.left_stick_y);
            robot.lift.manualControl(-gamepad2.right_stick_y);

            if (!scheduler.isBusy()) {
                // Intake Control
                // Intake Down and Spin
                if ((gamepad2.right_trigger > 0.1) && !hasWrongColor) {
                    robot.intake.spin.setPower(1);
                    intake.intakeDown();
                } else if ((gamepad2.left_trigger > 0.1)) {
                    // Eject the Specimen
                    robot.intake.spin.setPower(-1);
                } else {
                    // Intake Up and Stop Spin
                    robot.intake.spin.setPower(0);
                    robot.intake.intakeUp();
                }

                // Specimen Outtake
                if ((gamepad2.dpad_down)) {
                    robot.outtakeSpec();
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

//            Hang Section
//            if ((gamepad2.square) && !scheduler.isBusy()) {
//                scheduler.queueAction(robot.hang.hangIn());
//            }

//            if ((gamepad2.circle) && !scheduler.isBusy()) {
//                scheduler.queueAction(robot.hang.hangOut());
//            }

            if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1) {
                if (robot.intake.hasSample() && robot.intake.isRightColor() && !scheduler.isBusy()) {
                    scheduler.queueAction(robot.transfer());
                } else if (robot.intake.hasSample() &&
                        !robot.intake.isRightColor() &&
                        !scheduler.isBusy()) {
                    // TODO Move back into robot? Gamepad is not accessible as it is right now
                    scheduler.queueAction(
                            new ActionUtil.RunnableAction(() -> {
                                intake.intakeEject();
                                intake.spin.setPower(Intake.INTAKE_EJECT);
                                if (gamepad2.right_trigger < .1) {
                                    return true;
                                }
                                return false;
                            })
                    );
                }
            }

            // Toggle The arm between specimen score and specimen intake
            switch (specGrabState) {
                case IDLE:
                    specGrabTimer.reset();
                    if (gamepad2.left_bumper) {
                        if (arm.wristPosition == arm.WRIST_INTAKE ||
                                arm.wristPosition == arm.WRIST_MIDDLE ||
                                arm.wristPosition == arm.WRIST_SPECIMEN_GRAB
                        ) {
                            specGrabState = SPECGRAB.SPECLIFT;
                        }
                    }
                    break;
                case SPECLIFT:
                    arm.grab();
                    Lift.targetPosition = Lift.ARM_FLIP_BACK;
                    if (specGrabTimer.seconds() > 0.5) {
                        specGrabTimer.reset();
                        specGrabState = SPECGRAB.SPECFRONT;
                    }
                    break;
                case SPECFRONT:
                    scheduler.queueAction(
                            new SequentialAction(
                                    new InstantAction(arm::specIntake),
                                    new InstantAction(() -> {
                                        arm.drop();
                                    })
                            )
                    );
                    specGrabTimer.reset();
                    specGrabState = SPECGRAB.SPECBACK;
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
                    robot.arm.drop();
                    // If cross pressed and the grabber is open, close it
                } else if (compareDouble(Arm.OPEN, robot.arm.grabber.getPosition())) {
                    // If the wrist is in the bucket prime position, move it to the bucket drop position
                    if (arm.wristPosition == arm.WRIST_BUCKET_PRIME) {
                        arm.setPivot(arm.PIVOT_BUCKET);
                        arm.wrist.setPosition(arm.WRIST_BUCKET_DROP);
                        arm.wristPosition = arm.WRIST_BUCKET_DROP;
                    }
                    robot.arm.grab();
                }
                clawTimer.reset();
            }

            telemetry.update();
            scheduler.update();
            robot.update();
        }
    }

    public boolean compareDouble(double a, double b) {
        return Math.abs(a - b) < 0.0001;

    }
}
