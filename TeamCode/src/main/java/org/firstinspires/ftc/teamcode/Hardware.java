package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Hardware {
    public MecanumDrive drive;

    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx intake;
    public DcMotorEx shooter;
    public DcMotorEx liftEH;
    public DcMotorEx liftCH;
    public CRServo turret;
    public CRServo transfer1; // closest to intake
    public CRServo transfer2; // farther from intake
    public Servo hood;

    public static double kP = 0.5;
    public static double kD = 0.0;
    public static double kV = 0.0004;
    private final PIDFController.PIDCoefficients pidCoef = new PIDFController.PIDCoefficients();
    public PIDFController shooterPID;

    public static boolean tuneShooter = false;
    public boolean shoot = false;

    public static double targetVelocity = 0;

    public Hardware(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        liftEH = hardwareMap.get(DcMotorEx.class, "liftEH");
        liftCH = hardwareMap.get(DcMotorEx.class, "liftCH");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transfer1 = hardwareMap.get(CRServo.class, "transfer1");
        transfer2 = hardwareMap.get(CRServo.class, "transfer2");

        turret = hardwareMap.get(CRServo.class, "turret");
        hood = hardwareMap.get(Servo.class, "hood");

        pidCoef.kP = kP;
        pidCoef.kD = kD;

        shooterPID = new PIDFController(pidCoef);
    }

    public void update() {
        if (tuneShooter) {
            pidCoef.kP = kP;
            pidCoef.kD = kD;
            shooterPID = new PIDFController(pidCoef);
        }

        // Treat PID as velocity error controller
        double currentVel = shooter.getVelocity(); // ticks/sec
        shooterPID.targetPosition = targetVelocity; // target velocity in ticks/sec

        double pidOut = shooterPID.update(currentVel); // uses (target - current)
        double ffOut = targetVelocity * kV; // kV in power per ticks/sec

        double power = pidOut + ffOut;
        power = Math.max(-1.0, Math.min(1.0, power)); // clamp

        if (shoot) {
            shooter.setPower(power);
        } else {
            shooter.setPower(0.0);
        }
    }
}

