package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Hardware {
    public MecanumDrive drive;

    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx intake;
    public DcMotorEx turretEncoder; // reuse intake motor for turret encoder reading
    public DcMotorEx shooter;
    public DcMotorEx transfer;
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

    public static double targetVel = 0;

    public Hardware(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = hardwareMap.get(CRServo.class, "turret");
        turretEncoder = intake;; // reuse intake motor for turret encoder reading
        hood = hardwareMap.get(Servo.class, "hood");

        pidCoef.kP = kP;
        pidCoef.kD = kD;

        shooterPID = new PIDFController(pidCoef);
    }

    public void update() {
//        if (tuneShooter) {
//            pidCoef.kP = kP;
//            pidCoef.kD = kD;
//            shooterPID = new PIDFController(pidCoef);
//        }

        // Treat PID as velocity error controller
//        double currentVel = shooter.getVelocity(); // ticks/sec
//        shooterPID.targetPosition = targetVel; // target velocity in ticks/sec
//
//        double pidOut = shooterPID.update(currentVel); // uses (target - current)
//        double ffOut = targetVel * kV; // kV in power per ticks/sec
//
//        double power = pidOut + ffOut;
//        power = Math.max(-1.0, Math.min(1.0, power)); // clamp
//
//        if (shoot) {
//            shooter.setPower(power);
//        } else {
//            shooter.setPower(0.0);
//        }
    }
}

