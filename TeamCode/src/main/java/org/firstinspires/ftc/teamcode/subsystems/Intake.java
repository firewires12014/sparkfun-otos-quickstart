package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class Intake extends Hardware {

    public Intake(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void in() {
        intake.setPower(1);
    }

    public void out() {
        intake.setPower(-1);
    }

    public void stop() {
        intake.setPower(0);
    }

//    public Action intakeGateAction() {
//        return new SequentialAction(
//                new InstantAction(() -> in()),
//                new SleepAction(Constants.AUTO_INTAKE_TIME),
//                new InstantAction(() -> stop()));
//    }
}
