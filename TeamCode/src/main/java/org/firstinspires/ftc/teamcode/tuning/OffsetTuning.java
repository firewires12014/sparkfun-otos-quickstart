package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp
@Disabled
public class OffsetTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


            double CIRCUMFRENCE = 1.25984 * Math.PI;
            double TICKS_PER_REV = 2000;

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, new Pose2d(0, 0, 0));
            IMU imu = hardwareMap.get(IMU.class, "imu");


            waitForStart();
            double imuPrev = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double runningAngle = imuPrev;
            while (opModeIsActive()) {
                localizer.update();

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                double distance = localizer.driver.getEncoderX();
                double velocity = localizer.driver.getVelX();

                // imu angle shi
                double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double angle_delta = angleWrap(currentAngle - imuPrev);
                runningAngle += angle_delta;
                imuPrev = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Velocity Method
                double wheel_velocity = velocity;
                double wheel_velo_inch = (wheel_velocity * CIRCUMFRENCE) / TICKS_PER_REV;
                double imu_velocity = localizer.driver.getHeadingVelocity();
                double velo_radius = Math.abs(wheel_velo_inch / imu_velocity);

                // Arc Method
                double wheel_distance = distance;
                double wheel_distanec_inch = (distance * CIRCUMFRENCE) / TICKS_PER_REV;
//                double arc = wheel_distanec_inch;
                double arc = distance;

                double arc_radius = arc / runningAngle;

                // Find angular offset relative to center of frame
                double offset_theta = Math.acos(Range.clip(arc / (arc_radius * runningAngle), -1, 1));

                telemetry.addData("wheel velo inch", wheel_velo_inch);
                telemetry.addData("imu velo", imu_velocity);
                telemetry.addData("velocity radius", velo_radius);
                telemetry.addData("wheel inch", arc);
                telemetry.addData("running imu angle", runningAngle);
                telemetry.addData("arc radius", arc_radius);
                telemetry.addData("Angluar Offset", offset_theta);

                telemetry.update();
            }

    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}
