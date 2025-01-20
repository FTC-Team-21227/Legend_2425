package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AUTONRIGHT_4specpushTHREE")
//HARD 4 spec
public class AUTON2025REDRIGHT_V2Robot_7 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10.5, -63.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ARM1_V2Robot arm1 = new ARM1_V2Robot(hardwareMap);
        ARM2_V2Robot arm2 = new ARM2_V2Robot(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);
        CLAW_ANGLE claw_angle = new CLAW_ANGLE(hardwareMap);
        double firstSpecDistance = -38.5 + 0.198 * (88.72 - arm2.highRung2 - 0.75);
        double otherSpecDistance = -38 + 0.198 * (88.72 - arm2.highRung2);
//        pushing timing to the limits

        //4 spec auto attempt.
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(9,firstSpecDistance),Math.toRadians(90));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(9, firstSpecDistance, Math.toRadians(90))) //push colored samples
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(9,-42),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(39, -47),Math.toRadians(0))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(39,-25),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -19),Math.toRadians(0))
                .strafeTo(new Vector2d(51, -48))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(51, -25),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(64, -19),Math.toRadians(0))
                .strafeTo(new Vector2d(62, -48))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -25),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(72, -19),Math.toRadians(0));
                //improved, we want 1 s shape
//                .splineToConstantHeading(new Vector2d(9,-45),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(46.5, -27),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(53, -15),Math.toRadians(0)) //make u's of this
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(53, -45),Math.toRadians(-90)) //y value may need to be changed, push 1st
////                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(53, -25),Math.toRadians(60))
//                .splineToConstantHeading(new Vector2d(66, -12),Math.toRadians(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(65, -47),Math.toRadians(-90)) //push 2nd
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(66, -25),Math.toRadians(60))
//                .splineToConstantHeading(new Vector2d(69, -12),Math.toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(70, -19, Math.toRadians(90))) //go to second specimen
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(70,-33),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(40,-48,Math.toRadians(0)),Math.toRadians(180)) //improved to a single movement
                .strafeTo(new Vector2d(45,-52),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(44.5, -60, Math.toRadians(0))) //pick up and place second specimen
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(7,otherSpecDistance,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(7, otherSpecDistance, Math.toRadians(90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(7,-41.3),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(36,-56,Math.toRadians(0)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44.5,-60),Math.toRadians(0),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(44.5, -60, Math.toRadians(0))) //pick up and place third specimen
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(4,otherSpecDistance,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(4, otherSpecDistance, Math.toRadians(90))) //go to fourth specimen
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(4,-41.3),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(36,-60,Math.toRadians(0)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44.5,-60),Math.toRadians(0),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(44.5, -60, Math.toRadians(0))) //pick up and place fourth specimen
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(2,otherSpecDistance,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(2, otherSpecDistance, Math.toRadians(90))) //park
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(2,-41.3),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(30,-58,Math.toRadians(-25)),Math.toRadians(0));

        Actions.runBlocking(
            new SequentialAction(
                claw.closeClaw(),
                intake_angle.RotatePosition1(),
                claw_angle.forward()
            )
        );

        Action firstTrajectory = tab1.build();
        //Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        //Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
        Action eighthTrajectory = tab8.build();
        Action ninthTrajectory = tab9.build();
        Action tenthTrajectory = tab10.build();
        Action eleventhTrajectory = tab11.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //first specimen
                        new ParallelAction(
                                claw.closeClaw(),
                                intake_angle.RotatePosition0(),
                                arm1.liftRung(1.8),
                                arm2.liftRungFirst(1.8,0.3),
                                firstTrajectory
                        ),
                        claw.openClaw(),
                        //push colored samples
                        new ParallelAction(
                                thirdTrajectory,
                                arm1.waitLiftDown(),
                                arm2.waitLiftDown()
                        ),
                        //face the wall and go to second specimen
                        new ParallelAction(
                            fifthTrajectory,
                            arm1.waitLiftWall(1.5,1.5),
                            arm2.waitLiftWall(1.5,1.5)
                        ),
                        claw.closeClaw(),
                        //pick up and place second specimen
                        new ParallelAction(
                            arm1.waitLiftRung(0.3),
                            arm2.waitLiftRung(0.3),
                            sixthTrajectory
                        ),
                        //go to third specimen
                        claw.openClaw(),
                        new ParallelAction(
                            seventhTrajectory,
                            arm1.waitLiftWall(),
                            arm2.waitLiftWall()
                        ),
                        //pick up and place third specimen
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.waitLiftRung(0.3),
                            arm2.waitLiftRung(0.3),
                            eighthTrajectory
                        ),
                        //go to fourth specimen
                        claw.openClaw(),
                        new ParallelAction(
                            ninthTrajectory,
                            arm1.waitLiftWall(),
                            arm2.waitLiftWall()
                        ),
                        //pick up and place fourth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.waitLiftRung(0.3),
                            arm2.waitLiftRung(0.3),
                            tenthTrajectory
                        ),
                        //go park
                        claw.openClaw(),
                        new ParallelAction(
                            eleventhTrajectory,
                            arm1.waitLiftWall(),
                            arm2.waitLiftWall()
                        )
                )
        );
        PoseStorage.currentPose = new Pose2d(new Vector2d(drive.pose.position.x,drive.pose.position.y),-Math.PI/2+drive.pose.heading.toDouble());
    }
}