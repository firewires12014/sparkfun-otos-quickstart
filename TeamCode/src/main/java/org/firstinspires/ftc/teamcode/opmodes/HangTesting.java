package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
@TeleOp
//@Disabled
public class HangTesting extends LinearOpMode {
        public DcMotorEx leftBack, rightBack, lift1, lift2;
        public Servo leftPTO, leftRack, rightPTO, rightRack, gearBox;

        public static double leftPtoLock = .5;
        public static double leftPtoUnlock = .9;
        public static double rightPtoLock = .5;
        public static double rightPtoUnlock = .9;
        public static double leftRackUp = 0;
        public static double leftRackDown = .2;
        public static double rightRackUp = 0;
        public static double rightRackDown = .2;
        public static double gearBoxHigh = .49;
        public static double gearBoxLow = .54;

        @Override
        public void runOpMode () throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            lift1 = hardwareMap.get(DcMotorEx.class, "lift");
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift2.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            leftPTO = hardwareMap.get(Servo.class,"leftPTO");
            rightPTO = hardwareMap.get(Servo.class,"rightPTO");
            leftRack = hardwareMap.get(Servo.class, "leftRack");
            rightRack = hardwareMap.get(Servo.class, "rightRack");
            gearBox = hardwareMap.get(Servo.class, "gearBox");

            // Any pre start init
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
                // Hang

                // Raise and lower rack and pinion
                if (gamepad1.right_trigger > 0.1) {
                    rightRack.setPosition(rightRackUp);
                    leftRack.setPosition(leftRackUp);
                } else if (gamepad1.left_trigger > 0.1) {
                    rightRack.setPosition(rightRackDown);
                    leftRack.setPosition(leftRackDown);
                }

                // Lock and Unlock PTO
                if (gamepad1.triangle) {
                    rightPTO.setPosition(rightPtoUnlock);
                    leftPTO.setPosition(leftPtoUnlock);
                } else if (gamepad1.x) {
                    rightPTO.setPosition(rightPtoLock);
                    leftPTO.setPosition(leftPtoLock);
                }

                // Shift from High to low gear
                if (gamepad1.dpad_left) {
                    gearBox.setPosition(gearBoxLow);
                } else if (gamepad1.dpad_right) {
                    gearBox.setPosition(gearBoxHigh);
                }

                if (gamepad1.dpad_down) {
                    lift1.setPower(1);
                    lift2.setPower(1);
//                    leftBack.setPower(1);
//                    rightBack.setPower(1);
                } else if  (gamepad1.dpad_up) {
                    lift1.setPower(-1);
                    lift2.setPower(-1);
//                    leftBack.setPower(-1);
//                    rightBack.setPower(-1);
                } else {
                    lift1.setPower(0);
                    lift2.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                }


                telemetry.addData("Left Back Power", leftBack.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Right Back Power", rightBack.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("L1ft 1 Power", lift1.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.addData("Lift 2 Power", lift2.getCurrent(CurrentUnit.MILLIAMPS));
                telemetry.update();
            }
        }
    }

