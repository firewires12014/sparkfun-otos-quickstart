package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.messages.ActionUtil;

public class intake {
    public double open = 0;
    public double closed = 1;
    public Servo intakeServo;
    public double error = 0;
    public double kP = 0;
    public double pos = 1;

    public intake (HardwareMap hardwareMap){
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

    }
    public void update (){
    }

    public Action intakeOn () {
        return new ActionUtil.ServoPositionAction(intakeServo, open);
    }

    public Action intakeOff () {
        return new ActionUtil.ServoPositionAction(intakeServo, closed);
    }

    public Action Lift () {
        return null;
    }
    public Action diffy (){
        return null;
    }
    public Action extend (){
        return null;
    }

    public Action score () {
        return new SequentialAction(
                new ParallelAction(
                        Lift(),
                        new SequentialAction(
                                new SleepAction(5000),
                                diffy()
                        )
                ),
                extend()
        );

    }
}
