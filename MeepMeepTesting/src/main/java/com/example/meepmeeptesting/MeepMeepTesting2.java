package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();
        meepMeep.setAxesInterval(1);
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(11, 63.5, Math.toRadians(270)))//js testing splines out
                        .splineToSplineHeading(new Pose2d(46.5, 50.5, Math.toRadians(40)), Math.toRadians(0))
//                .lineToX(10)
//                .strafeTo(new Vector2d(5,72))
//                .strafeTo(new Vector2d(8, 114.5))
//                .strafeTo(new Vector2d(10.5, 114.5))//first sample
//                .waitSeconds(2.75)
//                .turnTo(Math.toRadians(140)) //face basket
//                .strafeTo(new Vector2d(0, 122))
//                .strafeToSplineHeading(new Vector2d(12,112),Math.toRadians(0))
//                //.turnTo(Math.toRadians(0))//get second sample
//                .strafeTo(new Vector2d(8.25,124.5)) //change
//                .waitSeconds(2.8)
//                .strafeToSplineHeading(new Vector2d(15,115),Math.toRadians(145)) //face basket
//                .strafeTo(new Vector2d(-1.5, 122))
//                .strafeToSplineHeading(new Vector2d(3, 116),0)
//                .waitSeconds(1)
//                .turn(Math.toRadians(0.1))
//                .waitSeconds(1)
//                .turn(Math.toRadians(0.1))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
