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

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ActionScheduler;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    enum CLAW {
        IDLE,
        DROP,
        GRAB
    }
    CLAW clawState = CLAW.IDLE;
    ElapsedTime clawTimer = new ElapsedTime();

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
            if (!scheduler.isBusy()) {
                robot.intake.manualControl(-gamepad2.left_stick_y);
                robot.lift.manualControl(-gamepad2.right_stick_y);

                // Intake Control
                // Intake Down and Spin
                if ((gamepad2.right_trigger > 0.1)) {
                    robot.intake.spin.setPower(1);
                    intake.intakeDown();
                } else if ((gamepad2.left_trigger > 0.1)) {
                    // Eject the Specimen
                    scheduler.queueAction(robot.eject());
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
            if (robot.intake.hasSample() && !robot.intake.isRightColor()) {
                scheduler.queueAction(robot.eject());
            }

            if (robot.intake.hasSample() && robot.intake.isRightColor()) {
                scheduler.queueAction(robot.transfer());
            }

//            Hang Section
//            if ((gamepad2.square) && !scheduler.isBusy()) {
//                scheduler.queueAction(robot.hang.hangIn());
//            }

//            if ((gamepad2.circle) && !scheduler.isBusy()) {
//                scheduler.queueAction(robot.hang.hangOut());
//            }

            // Toggle The arm between specimen score and specimen intake
            if (gamepad2.left_bumper) {
                if (arm.wristPosition == arm.WRIST_INTAKE ||
                        arm.wristPosition == arm.WRIST_MIDDLE ||
                        arm.wristPosition == arm.WRIST_SPECIMEN_GRAB
                ) {
                    arm.grab();
                    clawState = CLAW.GRAB;
                    scheduler.queueAction(
                            new SequentialAction(
                                    lift.setTargetPositionActionBlocking((int) Lift.ARM_FLIP_BACK),
                                    new SleepAction(.5),
                                    new InstantAction(arm::specIntake),
                                    new SleepAction(.5),
                                    new InstantAction(() -> {
                                        arm.drop();
                                        clawState = CLAW.DROP;
                                    }),
                                    lift.setTargetPositionActionBlocking((int) Lift.SPECIMEN_PICKUP)
                            )
                    );
                } else {
                    scheduler.queueAction(robot.specScore());
                    clawState = CLAW.DROP;
                }
            }

            // Toggle the arm between bucket score and bucket intake
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

            // Toggle the claw between open and closed
            switch (clawState) {
                case IDLE:
                    clawTimer.reset();
                    if (gamepad2.cross) clawState = CLAW.GRAB;
                    break;
                case GRAB:
                    robot.arm.grab();
                    if (gamepad2.cross && clawTimer.seconds() > 0.3) {
                        clawTimer.reset();
                        clawState = CLAW.DROP;
                    }
                    break;
                case DROP:
                    robot.arm.drop();
                    if (clawTimer.seconds() > 0.5) clawState = CLAW.IDLE;
                    break;
            }

            telemetry.update();
            scheduler.update();
            robot.update();
        }
    }
}
