package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

public class Drive extends Hardware {

    Localizer localizer;

    public Drive(HardwareMap hardwareMap) {
        super(hardwareMap);

        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, new Pose2d(0, 0, 0));
    }

    public Drive(HardwareMap hardwareMap, Pose2d startPose) {
        super(hardwareMap);

        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, startPose);
    }

    public void drive(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // Send calculated power to wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void update() {
        localizer.update();
    }

    public Pose2d getPose() {
        return localizer.getPose();
    }

    public void  setPose(Pose2d pose) {
        localizer.setPose(pose);
    }
}
