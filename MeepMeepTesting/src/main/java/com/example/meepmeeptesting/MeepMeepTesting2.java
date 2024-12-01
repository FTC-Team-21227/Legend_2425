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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        meepMeep.setAxesInterval(1);
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -63.3, Math.toRadians(90)))//js testing splines out
                        .splineToConstantHeading(new Vector2d(10,-40),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(10,-45),Math.toRadians(270))
                        //.splineToConstantHeading(new Vector2d(10,-55),Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(38, -10), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(48, -10), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(58, -55), Math.toRadians(270))
                        .strafeToSplineHeading(new Vector2d(20,-55),Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(50,-55),Math.toRadians(0))
                        .lineToX(45)
                        .splineToSplineHeading(new Pose2d(10,-40,Math.toRadians(90)),Math.toRadians(90))
                        .lineToY(-45)
                        .splineToSplineHeading(new Pose2d(50,-55,Math.toRadians(0)),Math.toRadians(0))
                        .lineToX(45)
                        .splineToSplineHeading(new Pose2d(10,-40,Math.toRadians(90)),Math.toRadians(90))
                        .lineToY(-45)
                        .splineToConstantHeading(new Vector2d(50,-55),Math.toRadians(0))
                        .build());

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(15, -63.3, Math.toRadians(90)))//js testing splines out
                        .strafeTo(new Vector2d(10,-40))
                        .strafeTo(new Vector2d(10,-50))
                        .strafeTo(new Vector2d(35,-50))
                .strafeTo(new Vector2d(35, -10))
                .strafeTo(new Vector2d(48, -10))
                .strafeTo(new Vector2d(48, -55))
                .strafeTo(new Vector2d(48, -10))
                .strafeTo(new Vector2d(56, -10))
                .strafeTo(new Vector2d(56, -55))
                        .strafeToSplineHeading(new Vector2d(20,-55),Math.toRadians(0))
                        .strafeTo(new Vector2d(30,-55))
                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(10,-55))
                        .strafeTo(new Vector2d(10,-40))
                        .strafeTo(new Vector2d(10,-50))
                        .strafeToSplineHeading(new Vector2d(20,-55),Math.toRadians(0))
                        .strafeTo(new Vector2d(30,-55))
                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(10,-55))
                        .strafeTo(new Vector2d(10,-40))
                        .strafeTo(new Vector2d(10,-50))
                        .strafeTo(new Vector2d(30,-55))
                .build());
        RoadRunnerBotEntity t1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        t1.runAction(t1.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))//js testing splines out
                .splineToConstantHeading(new Vector2d(50,50),Math.toRadians(90))
                .build());
        RoadRunnerBotEntity t2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        t2.runAction(t2.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))//js testing splines out
                .splineToConstantHeading(new Vector2d(50,50),Math.toRadians(120))
                .build());
        RoadRunnerBotEntity t3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        t3.runAction(t3.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))//js testing splines out
                .splineToConstantHeading(new Vector2d(50,50),Math.toRadians(150))
                .build());
        RoadRunnerBotEntity t4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        t4.runAction(t4.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))//js testing splines out
                .splineToConstantHeading(new Vector2d(50,50),Math.toRadians(135))
                .build());
        RoadRunnerBotEntity t5 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.35)
                .build();
        //        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
        t5.runAction(t5.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))//js testing splines out
                .splineToConstantHeading(new Vector2d(50,50),Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                //.addEntity(myBot2)
//                .addEntity(t1)
//                .addEntity(t2)
//                .addEntity(t3)
//                .addEntity(t4)
//                .addEntity(t5)

                .start();
    }
}
