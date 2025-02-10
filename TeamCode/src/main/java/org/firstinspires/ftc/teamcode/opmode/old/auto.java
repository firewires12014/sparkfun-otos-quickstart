    package org.firstinspires.ftc.teamcode.opmode.old;

    import static org.firstinspires.ftc.teamcode.subsystems.old.Intake.GEEKED;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.acmerobotics.roadrunner.InstantAction;
    import com.acmerobotics.roadrunner.ParallelAction;
    import com.acmerobotics.roadrunner.Pose2d;
    import com.acmerobotics.roadrunner.SleepAction;
    import com.acmerobotics.roadrunner.Vector2d;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

    import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.teamcode.subsystems.old.Intake;
    import org.firstinspires.ftc.teamcode.subsystems.old.Lift;
    import org.firstinspires.ftc.teamcode.subsystems.old.Outtake;
    import org.firstinspires.ftc.teamcode.subsystems.old.Robot;
    import org.firstinspires.ftc.teamcode.util.AutoActionScheduler;

    @Autonomous(name="Auto", group="Into the Deep")
    @Disabled
    public class auto extends LinearOpMode {
        Robot robot;
        AutoActionScheduler scheduler;
        Pose2d startingPosition;
        Pose2d preloadPosition;

        Pose2d preloadAndFirst = new Pose2d(new Vector2d(15.77, 21.82), Math.toRadians(-20.5));
        Pose2d second = new Pose2d(new Vector2d(15.98, 27.01), Math.toRadians(-8.69));

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            robot = new Robot(telemetry, hardwareMap);
            scheduler = new AutoActionScheduler(()-> {
                robot.update();

                update();
            });
            startingPosition = new Pose2d(0, 0, Math.toRadians(0));
            preloadPosition = new Pose2d(18, 20, Math.toRadians(-25));

            robot.outtake.hold();

            while (opModeInInit() && !isStopRequested()) {
                robot.intake.down.setPosition(Intake.fourbarResting);
                robot.intake.lock.setPosition(Intake.SOMETHING_IN_BETWEEN);
                Lift.targetPosition = 0;
                robot.lift.update();
            }

            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
                preload();

                //intakeAndScore(900);

                // Get First Sample

                scheduler.addAction(robot.transfer());

                scheduler.addAction(new SleepAction(0.25));

                scheduler.addAction(robot.outtakeBucketAuto(0.8, 3200));
                scheduler.addAction(new InstantAction(()-> robot.intake.spin.setPower(1)));
                scheduler.addAction(new SleepAction(1.3));
                scheduler.addAction(new InstantAction(()-> robot.intake.spin.setPower(0)));
                scheduler.addAction(robot.droppa());
                scheduler.addAction(robot.intake.fourbarOut());
                scheduler.addAction(robot.dropAndReturnAuto());
                scheduler.addAction(robot.intake.setTargetPositionAction(300));

                // Move to next cycle
                scheduler.addAction(robot.drive.actionBuilder(preloadAndFirst)
                        .strafeToLinearHeading(second.position, second.heading)
                        .build());

                scheduler.addAction(robot.getIntakeReady(900));
                scheduler.run();

                scheduler.addAction(robot.intake.setTargetPositionAction(1200));

                scheduler.addAction(robot.transfer());

                scheduler.addAction(new SleepAction(0.25));

                scheduler.addAction(robot.outtakeBucketAuto(0.8, 3200));
                scheduler.addAction(new InstantAction(()-> robot.intake.spin.setPower(1)));
                scheduler.addAction(new SleepAction(1.3));
                scheduler.addAction(new InstantAction(()-> robot.intake.spin.setPower(0)));
                scheduler.addAction(robot.droppa());
                scheduler.addAction(robot.intake.fourbarOut());
                scheduler.addAction(robot.dropAndReturnAuto());
                scheduler.addAction(robot.intake.setTargetPositionAction(300));


                scheduler.addAction(robot.drive.actionBuilder(second)
                        .turnTo(Math.toRadians(10))
                        .build());
//
                scheduler.addAction(robot.intake.fourbarOut());
                scheduler.addAction(new InstantAction(robot.intake::somethingInBetween));
                scheduler.addAction(robot.intake.autoExtend(0.5, 1000));

                scheduler.addAction(robot.intake.setTargetPositionAction(1200));
                scheduler.addAction(robot.drive.actionBuilder(second)
                        .turnTo(Math.toRadians(-10))
                        .build());

                scheduler.addAction(robot.transfer());

                scheduler.addAction(new SleepAction(0.25));

                scheduler.addAction(robot.outtakeBucketAuto(1.7, 3200));
                scheduler.addAction(new InstantAction(()-> robot.intake.spin.setPower(1)));
                scheduler.addAction(new SleepAction(0.8));
                scheduler.addAction(new InstantAction(()-> robot.intake.spin.setPower(0)));
                scheduler.addAction(robot.droppa());
                scheduler.addAction(robot.intake.fourbarOut());
                scheduler.addAction(robot.dropAndReturnAuto());
                scheduler.run();

//                scheduler.addAction(robot.endAuto(telemetry, 30));
//                scheduler.run();























// PARK
//                scheduler.addAction(robot.drive.actionBuilder(preloadPosition)
//                        .turnTo(Math.toRadians(180))
//                        .splineTo(new Vector2d(53.0943, -18), Math.toRadians(180))
//                        .build());
//                scheduler.addAction(robot.autoPark());

//                scheduler.addAction(robot.endAuto(telemetry, 30));
//                scheduler.run();
                return;
            }
        }

        public void preload () {

            scheduler.addAction(new ParallelAction(
                    robot.outtakeBucketAuto(0.7, 3200),
                    robot.intake.setTargetPositionAction(600),
                    robot.drive.actionBuilder(startingPosition)
                            .strafeToLinearHeading(preloadAndFirst.position, preloadAndFirst.heading)
                            .build(),
                    robot.intake.fourbarOut(),
                    new InstantAction(robot.intake::somethingInBetween)
            ));
            scheduler.addAction(new SleepAction(0.5));
            scheduler.addAction(new InstantAction(() -> robot.outtake.grab.setPosition(Outtake.GRAB_POSITION_CLOSED)));
            scheduler.addAction(robot.droppa());
            scheduler.addAction(new ParallelAction(robot.dropAndReturnAuto(), robot.getIntakeReady(300),
                    robot.drive.actionBuilder(preloadAndFirst)
                            .turnTo(4)
                            .build()
                    ));
            scheduler.run();
        }

        public void intakeAndScore(double extensionDistace) {
//            scheduler.addAction(robot.intake.fourbarOut());
//            scheduler.addAction(new InstantAction(robot.intake::somethingInBetween));
//            scheduler.addAction(robot.intake.autoExtend(0.5, extensionDistace));
//            scheduler.addAction(robot.intake.setTargetPositionAction(1200));
//            scheduler.addAction(new SleepAction(1));
            scheduler.addAction(robot.transfer());
            scheduler.addAction(new SleepAction(1));
            scheduler.addAction(robot.outtakeBucketAuto(0.8, 3400));
            scheduler.addAction(new SleepAction(1.3));
            scheduler.addAction(robot.droppa());
            scheduler.addAction(robot.intake.fourbarOut());
            scheduler.addAction(robot.dropAndReturnAuto());
            scheduler.addAction(robot.getIntakeReady(extensionDistace));
            scheduler.run();
        }

        public void returnLiftAndIntake () {
            scheduler.addAction(new InstantAction(() -> {
                Lift.PID_ENABLED = false;
                robot.lift.lift.setPower(-1);
                robot.lift.lift2.setPower(-1);

                robot.intake.extension.setPower(0);
                Intake.PID_ENABLED = true;

                Outtake.power = 1; // could cause issues if it isn't set to zero later
            }));
//        while ((robot.intake.downSensor.getDistance(DistanceUnit.MM) < 20) || !(robot.intake.extension.getCurrentPosition()> 1000)) {}
            //while (robot.intake.downSensor.getDistance(DistanceUnit.MM) > 20) {}
            scheduler.await(robot.intake.downSensor.getDistance(DistanceUnit.MM) < 20);

            scheduler.run();

            scheduler.addAction(robot.transfer());
            scheduler.addAction(new InstantAction(() -> {
                Lift.PID_ENABLED = true;
                robot.intake.spin.setPower(0);
            }));

            scheduler.await(robot.lift.lift.getCurrentPosition() < 100, robot.lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 6000);

//            while (!(robot.lift.lift.getCurrentPosition() < 100) || !(robot.lift.lift.getCurrent(CurrentUnit.MILLIAMPS) > 6000)) {
//                telemetry.addData("getCurrentPosition", robot.lift.lift.getCurrentPosition());
//                telemetry.addData("milliamps", robot.lift.lift.getCurrent(CurrentUnit.MILLIAMPS));
//                telemetry.update();
//            }
            scheduler.run();

        }

        public void scoreSecond () {
            scheduler.addAction(new InstantAction(() -> robot.intake.lock.setPosition(GEEKED)));
            scheduler.addAction(robot.outtakeBucket());
            scheduler.run();
        }

        public void update () {
            //robot.update();

//            //telemetry.addData("intakeDownSensor", robot.intake.downSensor.getDistance(DistanceUnit.MM));
//            telemetry.addData("Lift Encoder Position", robot.lift.lift.getCurrentPosition());
//            //telemetry.addData("Lift Encoder Position", robot.lift.lift2.getCurrentPosition());
//            telemetry.addData("Intake Extension", robot.intake.extension.getCurrentPosition());
//            telemetry.addData("Lift currnet", robot.lift.lift.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.update();
        }
    }







