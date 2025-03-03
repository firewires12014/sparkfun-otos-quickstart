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

        Pose2d sample3Bucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));

        Pose2d preloadSubmersible = new Pose2d(new Vector2d(-2.9, -29), Math.toRadians(90));
        Pose2d splineAwayFromSubmersible = new Pose2d(new Vector2d(39, -36), Math.toRadians(90));
        Pose2d splineNextToFirstSample = new Pose2d(new Vector2d(46, -17.4), Math.toRadians(90));
        Pose2d splineToFirstSample = new Pose2d(new Vector2d(57, -8), Math.toRadians(-90));
        Pose2d pushFirstSample = new Pose2d(new Vector2d(57, -46), Math.toRadians(-90));

        Pose2d splineNextToSecondSample = new Pose2d(new Vector2d(58, -17.4), Math.toRadians(90));
        Pose2d splineToSecondSample = new Pose2d(new Vector2d(72, -8), Math.toRadians(-90));
        Pose2d pushSecondSample = new Pose2d(new Vector2d(72, -50), Math.toRadians(-90));

        Pose2d preloadBucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));
        Pose2d sample1Bucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));
        Pose2d sample2Bucket = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));;
        Pose2d cycleBucket1 = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));;
        Pose2d cycleBucket2 = new Pose2d(new Vector2d(-60.62, -56.76), Math.toRadians(54.17));;

        Pose2d sample1Pose = new Pose2d(new Vector2d(-48, -42.09), Math.toRadians(92));
        Pose2d sample2Pose = new Pose2d(new Vector2d(-58.92, -42.09), Math.toRadians(92));
        Pose2d sample3Pose = new Pose2d(new Vector2d(-61.38, -40.92), Math.toRadians(114.80));
        //Pose2d sample3Pose = new Pose2d(new Vector2d(-67.27, -45.7), Math.toRadians(115.3));

        Pose2d subIntake1 = new Pose2d(-22, -12, Math.toRadians(0));
        Pose2d subIntake2 = new Pose2d(-20, -6, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setStartPose(new Pose2d(7.1, -64, Math.toRadians(90))





                )
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(sample3Bucket)
                .splineTo(subIntake1.position, Math.toRadians(0))
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