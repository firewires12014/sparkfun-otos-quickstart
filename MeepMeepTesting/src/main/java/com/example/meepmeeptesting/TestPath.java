package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestPath {
    // Set the offset values
    private static double offsetX;
    private static double offsetY;
    private static double offsetHeading;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(new Pose2d(7.1, -64, Math.toRadians(90))

                )
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(7.1, -64, Math.toRadians(90)))
               // robot.drive.actionBuilder(robot.drive.pose)

                                .strafeToLinearHeading(new Vector2d(-2.9, -29), Math.toRadians(90),
                                        new TranslationalVelConstraint(70.0),
                                        new ProfileAccelConstraint(-120, 100))
                               // .build()


                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(39, -36), Math.toRadians(0))
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(46, -17.4), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(57, -6), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(57, -46), Math.toRadians(-90))
                       // .build()


                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(50, -17.4), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(62, -15), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(63, -56), Math.toRadians(-90))
                        .build());
                       // .build());

//                .splineToConstantHeading(new Vector2d(54.1, -46), Math.toRadians(0))
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(67.1, -6), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(67.1, -46), Math.toRadians(0))
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(76.1, -6), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(75.1, -56), Math.toRadians(-90))
               // .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}