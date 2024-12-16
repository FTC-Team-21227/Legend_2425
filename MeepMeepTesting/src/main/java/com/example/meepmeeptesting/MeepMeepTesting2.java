package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -63.3, Math.toRadians(90)))
                .strafeTo(new Vector2d(10,-41.3))
                .setTangent(Math.toRadians(Math.toRadians(-30))) //improved, we want 1 s shape
                .splineToConstantHeading(new Vector2d(40, -30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -20),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50, -50),Math.toRadians(-90)) //y value may need to be changed
                .splineToConstantHeading(new Vector2d(50, -30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56,-20),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(62, -30),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(62, -50),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(62,-30,Math.toRadians(0)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(68,-20),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(74,-30),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(74,-60),Math.toRadians(-90))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(30,-60),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(40,-60),Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(5,-41.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-55,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(0,-41.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-55,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-5,-41.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-55,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-10,-41.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-55),Math.toRadians(0))
                .build());

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                .setConstraints(70,60,Math.toRadians(360),Math.toRadians(360),13.35)
                .build();
        myBot3.runAction(myBot3.getDrive().actionBuilder(new Pose2d(15,-63.3,Math.toRadians(90)))
                .strafeTo(new Vector2d(10,-41.3))
                .setTangent(Math.toRadians(Math.toRadians(-30))) //improved, we want 1 s shape
                .splineToConstantHeading(new Vector2d(40, -30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -20),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50,-30),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50, -50),Math.toRadians(-90)) //y value may need to be changed
                .splineToConstantHeading(new Vector2d(50, -30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56,-20),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(62, -30),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(62, -50),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(62,-30,Math.toRadians(180)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(68,-20),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(74,-30),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(74,-50),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-45,Math.toRadians(-90)),Math.toRadians(-90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(5,-39.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-45,Math.toRadians(-90)),Math.toRadians(-90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(0,-39.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-45,Math.toRadians(-90)),Math.toRadians(-90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-5,-39.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-45,Math.toRadians(-90)),Math.toRadians(-90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-5,-39.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-45),Math.toRadians(-90))
                .build());

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(15, -63.3, Math.toRadians(90)))
                                //.waitSeconds(1)
                .strafeTo(new Vector2d(10,-41.3))
                .setTangent(Math.toRadians(-45))
                //.splineToConstantHeading(new Vector2d(40,-53),Math.toRadians(90))
                //.setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50, -20),Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                //.strafeTo(new Vector2d(50, -53))
                .splineToConstantHeading(new Vector2d(50, -50),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(50, -32),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -20),Math.toRadians(0))
                .strafeTo(new Vector2d(62, -53))
                .strafeToSplineHeading(new Vector2d(40,-60),Math.toRadians(0))
                .waitSeconds(1)
//                .turnTo(Math.toRadians(90))
//                .strafeTo(new Vector2d(22,-55))
//                .strafeTo(new Vector2d(5,-55))
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(5,-41.3,Math.toRadians(90)),Math.toRadians(90))
//                .strafeTo(new Vector2d(5,-41.3))
                        .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-55,Math.toRadians(0)),Math.toRadians(0))
//                .strafeTo(new Vector2d(5,-55))
//                .turnTo(Math.toRadians(0))
//                .strafeTo(new Vector2d(40,-55))
                .waitSeconds(1)
//                .turnTo(Math.toRadians(90))
//                .strafeTo(new Vector2d(22,-55))
//                .strafeTo(new Vector2d(5,-55))
//                .strafeTo(new Vector2d(5,-41.3))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(0,-41.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
//                .turnTo(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(40,-55,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-5,-41.3,Math.toRadians(90)),Math.toRadians(90))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-55),Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .start();
    }
}
