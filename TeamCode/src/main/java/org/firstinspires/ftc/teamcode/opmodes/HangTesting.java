package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
            Robot robot = new Robot(hardwareMap);
            leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
            lift1 = hardwareMap.get(DcMotorEx.class, "lift");
            lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

            // Any pre start init
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
                // Hang
                if (gamepad1.cross) {
                    robot.lockPTO();
                }

                if (gamepad1.square) {
                    robot.unlockPTO();
                }

                if (gamepad1.dpad_left) {
                    robot.lowerRack();
                }

                if (gamepad1.dpad_right) {
                    robot.liftRack();
                }

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
                    lift1.setPower(00);
                    lift2.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                }

                leftBack.setPower(gamepad1.left_stick_y);
                rightBack.setPower(gamepad1.right_stick_y);

            }
        }
    }

