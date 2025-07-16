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
    //private TRANSFER_STATE = ;

    enum ROBOT_STATE {
        SCORE_HIGH_BUCKET,
        SCORE_LOW_BUCKET,
    }

    enum OPERATING_MODE {
        SPEC,
        SAMPLE,
        HANG,
    }

    public enum INTAKE_STATE {
        IN, // Pivot up, extension full retracted
        OUT, // Pivot horizontal, out slightly
        INTAKING, // Pivot down, extension doesn't matter
        OUTTAKING, // Pivot horizontal, extension doesn't matter
        EJECT, //Pivot blah blah stfu
        TRANSFER,
        IDLE
    }

    public enum FARM_STATE {
        IDLE,
        SPEC_INTAKE,
        LOW_BUCKET_SCORE,
        TRANSFER,
        SPEC_SCORE,

        SPEC_SCORE_LOW,
        BUCKET_SCORE,
        RETURN_LIFT
    }

    public enum TRANSFER_STATE {
        RETURNING,
        WAIT,
        GRAB,
        IDLE
    }

    public enum PTO_STATE {
        UNLOCK,
        LOCK,
        RETRY,
        IDLE
    }

    OPERATING_MODE operatingState = OPERATING_MODE.SAMPLE;
    INTAKE_STATE intakeState = INTAKE_STATE.IN;
    FARM_STATE farmState = FARM_STATE.TRANSFER;
    TRANSFER_STATE transferState = TRANSFER_STATE.IDLE;

    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime clawTimer = new ElapsedTime();
    ElapsedTime specTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime hangTimer = new ElapsedTime();
    ElapsedTime colorTimer = new ElapsedTime();
    ElapsedTime ptoTimer = new ElapsedTime();

    boolean firstRun = true;

    double prevLoop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0,0), LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        prevLoop = System.nanoTime() / 1e9;
        intakeTimer.reset();
        transferTimer.reset();
        colorTimer.reset();
        hangTimer.reset();
        ptoTimer.reset();
        operatingState = OPERATING_MODE.SAMPLE;
        robot.farm.setTransfer();
        robot.setGearBoxHigh();
        robot.unlockPTO();
        robot.setColorRed();


        while (opModeIsActive() && !isStopRequested()) {
            robot.clearBulkCache(); // Do once per loop

            // Calum

            // Drivetrain

            gamepad1.setLedColor(255, 255, 0, -1);

            if (!operatingState.equals(OPERATING_MODE.HANG)) {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));
            }

            if (gamepad1.left_trigger > 0.1) {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                gamepad1.left_trigger
                        ),
                        0
                ));
            }

            if (gamepad1.right_trigger > 0.1) {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0,
                                -gamepad1.right_trigger
                        ),
                        0
                ));
            }

            // Hang

            if (gamepad1.touchpad  && hangTimer.seconds() > 0.5) {
                if (!operatingState.equals(OPERATING_MODE.HANG)) {
                    FArm.PID_ENABLED = false;
                    farmState = FARM_STATE.SPEC_SCORE;
//                    robot.lowerRack();
//                    robot.setGearBoxLow();
                    operatingState = OPERATING_MODE.HANG;
                } else {
                    FArm.PID_ENABLED = true;
                    robot.liftRack();
                    robot.setGearBoxHigh();
                    operatingState = OPERATING_MODE.SAMPLE;
                }

                hangTimer.reset();
            }
            if (gamepad1.dpad_left) {
                robot.lockPTO();
            }

            if (gamepad1.dpad_right) {
                robot.unlockPTO();
            }

            // Wasi
            // Lift

            //WHY THE FUCK DOES THIS SAY ITS ALWAYS TRUE IT LITERALLY ISNT I HATE PROGRAMMING THIS SHIT IS AWFUL
            // skill issue
            if (operatingState.equals(OPERATING_MODE.HANG)) {
                robot.farm.hangManualControl(-gamepad2.right_stick_y);
            } else {
                robot.farm.manualControl(-gamepad2.right_stick_y);
            }

            if (Math.abs(-gamepad2.right_stick_y) > FArm.joystickDeadzone) {
                farmState = FARM_STATE.IDLE;
            }
            //IF I SEE ONE MORE FUCKING YELLOW SQUIGGLE LINE IM THROWING THIS LAPTOP

            // Intake

            robot.intake.manualControl(-gamepad2.left_stick_y);

            if (operatingState.equals(OPERATING_MODE.SPEC) && intakeState.equals(INTAKE_STATE.TRANSFER)) {
                intakeState = INTAKE_STATE.IDLE;
                transferState = TRANSFER_STATE.IDLE;
//                farmState = FARM_STATE.IDLE;
                robot.intake.intakeHorizontal();
                robot.intake.stopIntake();
            }

            if (gamepad2.right_trigger > 0.1) {
                intakeState = INTAKE_STATE.INTAKING;
            } else if (gamepad2.left_trigger > 0.1) {
                intakeState = INTAKE_STATE.OUTTAKING;
            } else {
                robot.intake.stopIntake();
                if (transferState.equals(TRANSFER_STATE.IDLE)) {
                    if (robot.intake.isOuttakeOut()) intakeState = INTAKE_STATE.OUT;
                    else intakeState = INTAKE_STATE.IN;
                }
            }
            //this state nonsense is pmo

            if (gamepad2.touchpad && colorTimer.seconds() > 1) {
                if (robot.selected_color.equalsIgnoreCase("RED")) {
                    robot.setColorBlue();
                    gamepad2.setLedColor(0, 0, 255, -1);
                } else {
                    robot.setColorRed();
                    gamepad2.setLedColor(255,0,0,-1);
                }
                colorTimer.reset();
            }

            if (intakeState.equals(INTAKE_STATE.INTAKING) && robot.hasSample()) {
                    intakeState = INTAKE_STATE.TRANSFER;
                    farmState = FARM_STATE.TRANSFER;
                    transferState = TRANSFER_STATE.RETURNING;
            }

            //Outtake

            // Still figure out transfer
            // assuming it transfers (grabs block that's it) then when intake is in manual open claw
            if (!Intake.PID_ENABLED && !robot.intake.holdIn) robot.farm.drop(); // make add a condition for spec (ion know when it would ever be the case)

            if (gamepad2.left_bumper) {
                robot.farm.grab.setPosition(FArm.specOpen);
                farmState = FARM_STATE.SPEC_INTAKE;
                if (clawTimer.seconds() > .5 && operatingState.equals(OPERATING_MODE.SPEC)){
//                    operatingState = OPERATING_MODE.SAMPLE;
                    clawTimer.reset();
                }
                else {
                    operatingState = OPERATING_MODE.SPEC;
                    clawTimer.reset();
                }
            }

            if (gamepad2.square) {
                farmState = FARM_STATE.SPEC_SCORE_LOW;
                operatingState = OPERATING_MODE.SAMPLE;
            }

            if (gamepad2.dpad_up && transferState.equals(TRANSFER_STATE.IDLE)) {
                farmState = FARM_STATE.BUCKET_SCORE;
                operatingState = OPERATING_MODE.SAMPLE;
            }

            if (gamepad2.dpad_left) {
                farmState = FARM_STATE.LOW_BUCKET_SCORE;
                operatingState = OPERATING_MODE.SAMPLE;
            }

            if (gamepad2.dpad_down) {
                farmState = FARM_STATE.SPEC_SCORE;
            }

            if (gamepad2.circle) {
                FArm.wristTransfer = .35;
            }

            if (operatingState.equals(OPERATING_MODE.SPEC)) {
               if (robot.farm.hasSpec() && robot.farm.isClawOpen()) {
                   robot.farm.close();
                   specTimer.reset();
               }

               if (specTimer.seconds() > .2 && !robot.farm.isClawOpen()) {
                   farmState = FARM_STATE.SPEC_SCORE;
                }

                if (specTimer.seconds() > .4 && robot.farm.isClawOpen() && farmState.equals(FARM_STATE.SPEC_SCORE)) {
                    farmState = FARM_STATE.SPEC_INTAKE;
                }

            }

            if (gamepad2.right_bumper) {
                robot.farm.setTransfer();
                farmState = FARM_STATE.RETURN_LIFT;
                operatingState = OPERATING_MODE.SAMPLE;
            }

            if (gamepad2.cross && clawTimer.seconds() > 0.3) {
                if (robot.farm.isClawOpen()) robot.farm.close();
                else {
                    robot.farm.drop();
                    if (operatingState.equals(OPERATING_MODE.SPEC)) {
                        specTimer.reset();
                    }
                }
                clawTimer.reset();
            }

            if (gamepad2.triangle) {
                robot.intake.resetEncoder();
                robot.farm.resetEncoder();
            }

            switch (farmState) {
                case IDLE:
                    break;
                case SPEC_INTAKE:
                    robot.farm.setSpecIntake();
                    robot.farm.setSpecIntake();
                    break;
                case TRANSFER:
                    // Is in state
                    break;
                case SPEC_SCORE:
                    robot.farm.setSpecScore();
                    break;
                case SPEC_SCORE_LOW:
                    robot.farm.setSpecScoreLow();
                    break;
                case BUCKET_SCORE:
                    robot.farm.setBucketScore();
                    break;
                case LOW_BUCKET_SCORE:
                    robot.farm.setBucketScore(true);
                    break;
                case RETURN_LIFT:
                    robot.turnOffLight();
//                    FArm.PID_ENABLED = false;
//                    robot.farm.lift.setPower(-1);
//                    robot.farm.lift2.setPower(-1);
                    FArm.targetPosition = 0;

                    if (robot.farm.lift.getCurrentPosition() < 1000) {
//                        robot.farm.resetEncoder();
                        FArm.targetPosition = 0;
                        FArm.PID_ENABLED = true;

                        farmState = FARM_STATE.TRANSFER;
                    }
            }

            switch (intakeState) {
                case IDLE:
                    break;
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
                    Intake.PID_ENABLED = false;
                    break;
                    //i mightve just cooked everything with this eject state idek what im doing
                case EJECT:
                    intakeTimer.reset();
                    robot.intake.intakeHorizontal();
                    if (intakeTimer.seconds() > 0.2) {
                        robot.intake.spin.setPower(.5);
                        intakeState = INTAKE_STATE.INTAKING;
                    }
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
                    // dumbass made an error
                    break;
            }

//            //this shit does not have to be an fsm but ion know what else to do
//            switch(ptoState) {
//                case UNLOCK:
//                    robot.unlockPTO();
//                    ptoState = PTO_STATE.IDLE;
//                    break;
//                case LOCK:
//                    robot.lockPTO();
//                    if (!(robot.isRightPtoLocked()) || !(robot.isLeftPtoLocked())) {
//                        ptoState = PTO_STATE.RETRY;
//                    } else {
//                        ptoState = PTO_STATE.IDLE;
//                    }
//                    break;
//                case RETRY:
//                    if (!(robot.isRightPtoLocked())) {
//                        ptoTimer.reset();
//                        robot.rightUnlockPTO();
//                        if (ptoTimer.seconds() > 0.5) {
//                            robot.rightLockPTO();
//                            ptoTimer.reset();
//                        }
//                    }
//                    if (!(robot.isLeftPtoLocked())) {
//                        ptoTimer.reset();
//                        robot.leftUnlockPTO();
//                        if (ptoTimer.seconds() > 0.5) {
//                            robot.leftLockPTO();
//                            ptoTimer.reset();
//                        }
//                    }
//                    ptoState = PTO_STATE.LOCK;
//                    break;
//                case IDLE:
//                    break;
//            }

            robot.update();
            if (operatingState == OPERATING_MODE.SAMPLE) {
                robot.farm.updatePID(0.00015, 0, 1.5e-8);
            } else {
                robot.farm.updatePID(0.00015, 0.0001, 1.5e-8);
            }

            telemetry.addData("Beam Brake State: " , robot.farm.hasSpec());
            telemetry.addLine("---Intake---");
            telemetry.addData("State", intakeState);
            telemetry.addLine("---FArm---");
            telemetry.addData("State", farmState);
            telemetry.addData("Target Position:", FArm.targetPosition);
            telemetry.addData("Actual Position", robot.farm.lift.getCurrentPosition());
            telemetry.addData("Lift Power", robot.farm.lift.getPower());
            telemetry.addData("Lift 1 Power:", robot.farm.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Lift 2 Power:", robot.farm.lift2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Transfer State", transferState);
            telemetry.addData("Sample Color Red", robot.colorValueRed);
            telemetry.addData("Sample Color Blue", robot.colorValueBlue);
            telemetry.addData("Sample Color Green", robot.colorValueGreen);
            telemetry.addData("Operating State", operatingState);
            telemetry.addData("intakeCurrent",robot.intake.extension.getCurrent(CurrentUnit.MILLIAMPS));
            loopTimeMeasurement(telemetry); // replaces telemetry.update()
        }
        robot.endOpmode();
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
