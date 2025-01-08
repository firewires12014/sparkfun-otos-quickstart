package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDFController;

public class Lift {
    public DcMotorEx lift;
    public DigitalChannel limitSwitch;

    PIDFController controller;
    PIDFController.PIDCoefficients coefficients;

    public static double liftPosition = 0;

    public static double ff = 0;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        //limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
    }

    public void update () {
       // controller.targetPosition = liftPosition;
        //double output = controller.update(liftPosition) + ff;
        //lift.setPower(output);

    }

    public Action getHangReady () {
        return new InstantAction(()-> {liftPosition = 100;});
    }
}
