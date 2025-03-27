package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
@TeleOp
//@Disabled
public class HangTesting extends LinearOpMode {
        public DcMotorEx leftBack, rightBack, lift1, lift2;

        @Override
        public void runOpMode () throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            lift1 = hardwareMap.get(DcMotorEx.class, "lift");
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
            lift2.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

//            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//            lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//            lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            // Any pre start init
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
                // Hang
                if (gamepad1.dpad_up) {
                    lift1.setPower(1);
                    lift2.setPower(1);
                    leftBack.setPower(1);
                    rightBack.setPower(1);
                } else if  (gamepad1.dpad_down) {
                    lift1.setPower(-1);
                    lift2.setPower(-1);
                    leftBack.setPower(-1);
                    rightBack.setPower(-1);
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

