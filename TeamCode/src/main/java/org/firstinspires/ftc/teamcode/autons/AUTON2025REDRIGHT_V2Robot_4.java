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

//@Autonomous(name = "AUTONRIGHT_V2Robot_5spec")
//5 spec with new V2 Robot, using a different wall pickup position.
public class AUTON2025REDRIGHT_V2Robot_4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(15, -63.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_V2Robot arm1 = new ARM1_V2Robot(hardwareMap);
        ARM2_V2Robot arm2 = new ARM2_V2Robot(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);
        CLAW_ANGLE claw_angle = new CLAW_ANGLE(hardwareMap);
        //5 spec auto attempt.
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(9,-41.3),Math.toRadians(90));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(drive.pose) //push colored samples
                .setTangent(Math.toRadians(-90))
                //improved, we want 1 s shape
                .splineToConstantHeading(new Vector2d(10,-45),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(46.5, -27),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -15),Math.toRadians(0))
                .strafeTo(new Vector2d(53, -50))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -25),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62,-15),Math.toRadians(0))
                .strafeTo(new Vector2d(62, -50))
                .setTangent(Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(56,-20),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(72,-15),Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(72,-50),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(72,-58.5),Math.toRadians(-90),new TranslationalVelConstraint(10));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(drive.pose) //pick up and place second specimen
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(125))
                .splineToLinearHeading(new Pose2d(8,-38.7,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(drive.pose) //go to third specimen
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(55,-58.5,Math.toRadians(90)),Math.toRadians(-15));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(drive.pose) //pick up and place second specimen
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(125))
                .splineToLinearHeading(new Pose2d(7,-38.7,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(drive.pose) //go to third specimen
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(55,-58.5,Math.toRadians(90)),Math.toRadians(-15));
        TrajectoryActionBuilder tab10 = drive.actionBuilder(drive.pose) //pick up and place second specimen
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(125))
                .splineToLinearHeading(new Pose2d(6,-38.7,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab11 = drive.actionBuilder(drive.pose) //go to third specimen
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(55,-58.5,Math.toRadians(90)),Math.toRadians(-15));
        TrajectoryActionBuilder tab12 = drive.actionBuilder(drive.pose) //pick up and place fifth specimen
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(125))
                .splineToLinearHeading(new Pose2d(5,-38.7,Math.toRadians(90)),Math.toRadians(90));
        TrajectoryActionBuilder tab13 = drive.actionBuilder(drive.pose) //park
                .setTangent(Math.toRadians(-120))
                .splineToLinearHeading(new Pose2d(35,-60,Math.toRadians(0)),Math.toRadians(0));

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
//        Action fifthTrajectory = tab5.build();
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
                                arm1.liftRung(1.5),
                                arm2.liftRung(1.5),
                                firstTrajectory
                        ),
                        claw.openClaw(),
                        //push colored samples; face the wall and go to second specimen
                        new ParallelAction(
                                thirdTrajectory,
                                arm1.waitLiftWall2(6),
                                arm2.waitLiftWall2(6),
                                claw_angle.backward()
                        ),
                        claw.closeClaw(),
                        //pick up and place second specimen
                        new ParallelAction(
                                claw_angle.forward(),
                                arm1.waitLiftRung(0.5),
                                arm2.waitLiftRung(0.5),
                                sixthTrajectory
                        ),
                        //go to third specimen
                        claw.openClaw(),
                        new ParallelAction(
                                claw_angle.backward(),
                                seventhTrajectory,
                                arm1.waitLiftWall(),
                                arm2.waitLiftWall()
                        ),
                        //pick up and place third specimen
                        claw.closeClaw(),
                        new ParallelAction(
                                claw_angle.forward(),
                                arm1.waitLiftRung(0.5),
                                arm2.waitLiftRung(0.5),
                                eighthTrajectory
                        ),
                        //go to fourth specimen
                        claw.openClaw(),
                        new ParallelAction(
                                claw_angle.backward(),
                                ninthTrajectory,
                                arm1.waitLiftWall(),
                                arm2.waitLiftWall()
                        ),
                        //pick up and place fourth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                                claw_angle.forward(),
                                arm1.waitLiftRung(0.5),
                                arm2.waitLiftRung(0.5),
                                tenthTrajectory
                        ),
                        //go to fifth specimen
                        claw.openClaw(),
                        new ParallelAction(
                                claw_angle.backward(),
                                eleventhTrajectory,
                                arm1.waitLiftWall(),
                                arm2.waitLiftWall()
                        ),
                        //pick up and place fifth specimen
                        claw.closeClaw(),
                        new ParallelAction(
                                claw_angle.forward(),
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
