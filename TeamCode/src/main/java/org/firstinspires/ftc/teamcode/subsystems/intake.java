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
    public DcMotorEx motorMotor;
    public double error = 0;
    public double kP = 0;
    public double pos = 1;

    public intake (HardwareMap hardwareMap){
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        motorMotor = hardwareMap.get(DcMotorEx.class, "left_front");

    }
    public void update (){
        motorMotor.getCurrentPosition();
        error = pos - motorMotor.getCurrentPosition();
        motorMotor.setPower(error*kP);
    }
    public Action moveMotorMotor (double pos) {
        this.pos = pos;
        return new ActionUtil.RunnableAction(() -> {
            if(motorMotor.getCurrentPosition() > pos){
                error = 0;
                this.pos = motorMotor.getCurrentPosition();
                return true;
            }
            else return false;
        });
    }
    public Action intakeOn () {
        return new ActionUtil.RunnableAction(() -> {
            intakeServo.setPosition(open);
            return (true);
        });
    }
    public Action intakeYay () {
        return new ActionUtil.ServoPositionAction(intakeServo, 0.5);
    }

    public Action intakeOff () {
        return new ActionUtil.RunnableAction(() -> {
            intakeServo.setPosition(closed);
            return (true);
            });
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
