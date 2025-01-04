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

//@Autonomous(name = "AUTONRIGHT_10_5specpusharm")
//5 spec with timesave from pushing samples with arm
public class AUTON2025REDRIGHT_10 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(15, -63.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        //5 spec auto attempt, pushing the samples with arm.
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                //.waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10,-41.3),Math.toRadians(90));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, -41.3, Math.toRadians(90))) //push colored samples WITH ARM
                .setTangent(Math.toRadians(-90))
                //improved, we want 1 s shape
                .splineToConstantHeading(new Vector2d(10,-45),Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(28, -40,Math.toRadians(30)),Math.toRadians(-90)); //CHANGE PUSHING by changing x,y pos, changing angle of attack, changing amount of x moved back
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(28, -40, Math.toRadians(30))) //push colored samples WITH ARM
                //improved, we want 1 s shape
                .splineToSplineHeading(new Pose2d(27,-48,Math.toRadians(0)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(28,-56,Math.toRadians(-30)),Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(28,-48,Math.toRadians(30)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(38, -40),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(37,-48,Math.toRadians(0)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(38,-56,Math.toRadians(-30)),Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(38,-48,Math.toRadians(30)),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48, -40),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(47,-48,Math.toRadians(0)),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(48,-66,Math.toRadians(0)),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(45,-68),Math.toRadians(-90));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(43, -66, Math.toRadians(0))) //go to second specimen
                .strafeTo(new Vector2d(44.5,-65.5),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(44, -66, Math.toRadians(0))) //pick up and place second specimen
                .waitSeconds(0.7) //CHANGED: reduced delay of movement (saves 1 s total)
                .setTangent(Math.toRadians(140)) //CHANGED: diagonal initial tangent should make the robot follow a tighter curve.
                .splineToLinearHeading(new Pose2d(5,-39.3,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(5, -39.3, Math.toRadians(90))) //go to third specimen
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(38,-60,Math.toRadians(0)),Math.toRadians(0)) //CHANGED: robot returns to the wall at an angle as well.
                .splineToConstantHeading(new Vector2d(44.5,-60),Math.toRadians(0),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(44, -60, Math.toRadians(0))) //pick up and place third specimen
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(2,-39.3,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(2, -39.3, Math.toRadians(90))) //go to fourth specimen
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(38,-60,Math.toRadians(0)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44.5,-60),Math.toRadians(0),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(44, -60, Math.toRadians(0))) //pick up and place fourth specimen
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(0,-39.3,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(2, -39.3, Math.toRadians(90))) //go to fifth specimen
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(38,-60,Math.toRadians(0)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44.5,-60),Math.toRadians(0),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab12 = drive.actionBuilder(new Pose2d(44, -60, Math.toRadians(0))) //pick up and place fifth specimen
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(0,-39.3,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab13 = drive.actionBuilder(new Pose2d(0, -39.3, Math.toRadians(90))) //park
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(35,-60,Math.toRadians(0)),Math.toRadians(0));

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        intake_angle.RotatePosition1()
                )
        );

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        //Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
        Action eighthTrajectory = tab8.build();
        Action ninthTrajectory = tab9.build();
        Action tenthTrajectory = tab10.build();
        Action eleventhTrajectory = tab11.build();
        Action twelfthTrajectory = tab12.build();
        Action thirteenthTrajectory = tab13.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        //first specimen
                        new ParallelAction(
                                claw.closeClaw(),
                                intake_angle.RotatePosition0(),
                                arm1.liftRung(1.3),
                                arm2.liftRung(1.3),
                                firstTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
                            secondTrajectory,
                                arm1.waitLiftFloor(1,2),
                                arm2.waitLiftFloor(1,2)
                                ),
                        //push colored samples WITH ARM
                        new ParallelAction(
                                thirdTrajectory,
                                claw.closeClaw()
                        ),
                        //face the wall and go to second specimen
                        new ParallelAction(
                                fifthTrajectory,
                                arm1.waitLiftWall(0.5,1),
                                arm2.waitLiftWall(0.5,1)
                        ),
                        claw.closeClaw(),
                        //pick up and place second specimen
                        new ParallelAction(
                                arm1.waitLiftRung(0.5),
                                arm2.waitLiftRung(0.5),
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
                                arm1.waitLiftRung(0.5),
                                arm2.waitLiftRung(0.5),
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
                                arm1.waitLiftRung(0.5),
                                arm2.waitLiftRung(0.5),
                                tenthTrajectory
                        ),
                        //go to fifth specimen
                        claw.openClaw(),
                        new ParallelAction(
                                eleventhTrajectory,
                                arm1.waitLiftWall(),
                                arm2.waitLiftWall()
                        ),
                        //pick up and place fifth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.waitLiftRung(0.5),
                                arm2.waitLiftRung(0.5),
                                twelfthTrajectory
                        ),
                        //go park
                        claw.openClaw(),
                        new ParallelAction(
                                thirteenthTrajectory,
                                arm1.waitLiftFloor(),
                                arm2.waitLiftFloor()
                        )
                )
        );
        PoseStorage.currentPose = new Pose2d(new Vector2d(drive.pose.position.x,drive.pose.position.y),Math.toRadians(-Math.PI/2+drive.pose.heading.toDouble()));
    }
}
