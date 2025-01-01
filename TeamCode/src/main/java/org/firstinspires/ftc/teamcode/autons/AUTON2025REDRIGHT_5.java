package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Autonomous(name = "AUTONRIGHT_5")
//3 specimen auto
public class AUTON2025REDRIGHT_5 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(15, -63.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(10,-41.3));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, -41.3, Math.toRadians(90))) //pull first specimen
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(40,-53),Math.toRadians(0));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(40, -53, Math.toRadians(90))) //push colored samples
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -30),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50, -20),Math.toRadians(0))
                .strafeTo(new Vector2d(50, -53))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50, -32),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62, -20),Math.toRadians(0))
                .strafeTo(new Vector2d(62, -53));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(62, -53, Math.toRadians(90))) //face the wall for second specimen
                .strafeTo(new Vector2d(30,-50))
                .turnTo(Math.toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(30, -50, Math.toRadians(0))) //go to second specimen
                .strafeTo(new Vector2d(40,-60));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(40, -60, Math.toRadians(0))) //pick up and place second specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(0,-39.3,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(0, -39.3, Math.toRadians(90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(42.5,-64,Math.toRadians(0)),Math.toRadians(0));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(42.5, -64, Math.toRadians(0))) //pick up and place third specimen
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-5,-39.3,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(-5, -39.3, Math.toRadians(90))) //park
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(45,-55,Math.toRadians(90)),Math.toRadians(0));

        claw.closeClaw();
        intake_angle.RotatePosition1();

        waitForStart();

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
        Action eighthTrajectory = tab8.build();
        Action ninthTrajectory = tab9.build();



        Actions.runBlocking(
                new SequentialAction(
                        //first specimen
                        new ParallelAction(
                                claw.closeClaw(),
                                intake_angle.RotatePosition0(),
                                arm1.liftRung(),
                                arm2.liftRung(),
                                firstTrajectory
                        ),
                        //pull first specimen
                        claw.openClaw(),
                        secondTrajectory,
                        //push colored samples
                        new ParallelAction(
                                arm1.waitLiftDown(),
                                arm2.waitLiftDown(),
                                thirdTrajectory
                        ),
                        //face the wall for second specimen
                        new ParallelAction(
                            fourthTrajectory,
                            arm1.liftWall(),
                            arm2.liftWall()
                        ),
                        //go to second specimen
                        fifthTrajectory,
                        claw.closeClaw(),
                        //waitSecond,
                        //pick up and place second specimen
                        new ParallelAction(
                            arm1.waitLiftRung(),
                            arm2.waitLiftRung(),
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
                        //waitSecond,
                        new ParallelAction(
                            eighthTrajectory,
                            arm1.waitLiftRung(),
                            arm2.waitLiftRung()
                        ),
                        //go park
                        claw.openClaw(),
                        new ParallelAction(
                            ninthTrajectory,
                            arm1.waitLiftDown(),
                            arm2.waitLiftDown(),
                            intake_angle.RotatePosition1()
                        )
                )
        );
    }
}
