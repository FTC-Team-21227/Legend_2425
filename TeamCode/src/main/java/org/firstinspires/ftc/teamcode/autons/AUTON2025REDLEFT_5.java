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

//@Autonomous(name = "AUTONLEFT_5")
public class AUTON2025REDLEFT_5 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(2.5, 123, Math.toRadians(140)), Math.toRadians(30));// loaded sample go to basket
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(4, 123, Math.toRadians(140)))
//                .strafeToLinearHeading(new Vector2d(12, 111), Math.toRadians(0));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(2.5, 123, Math.toRadians(140)))
                .splineToSplineHeading(new Pose2d(8, 113, Math.toRadians(0)), Math.toRadians(0)) //get 1st sample from the left side
                        .strafeTo(new Vector2d(10.5, 113)); //get 1st sample
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(10.5, 113, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(7, 90), Math.toRadians(140)) //go away from wall bec arms lifting
                .strafeTo(new Vector2d(3, 123)); //go to basket
//        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(4, 123, Math.toRadians(140)))
//                .strafeTo(new Vector2d(12, 111));//go away from while lowering arms
//                .splineToLinearHeading(new Pose2d(12, 124.5, Math.toRadians(0)), Math.toRadians(0)) //get 2nd sample
//                .strafeTo(new Vector2d(8.5, 124.5)); //go forward to grab 2nd sample
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(13, 124.5, Math.toRadians(0)))
//                .strafeTo(new Vector2d(10.5, 98)) //go away from basket/wall for space to turn
//                .strafeToLinearHeading(new Vector2d(4, 123), Math.toRadians(140));//go to basket
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(4,123, Math.toRadians(140)))
//                .strafeTo(new Vector2d(12, 111))//go away from while lowering arms
//                .splineToLinearHeading(new Pose2d(8.5, 133, Math.toRadians(0)), Math.toRadians(30)) //get 3rd sample
//                .strafeTo(new Vector2d(10.5, 134.5)); //go forward to grab 3rd sample
//        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(10.5,134.5, Math.toRadians(30)))
//                .strafeTo(new Vector2d(51, 46))
//                .strafeToLinearHeading(new Vector2d(5.5, 95), Math.toRadians(140)) //go away from wall bec arms lifting
//                .strafeTo(new Vector2d(4, 123)); //go to basket
//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(4,123, Math.toRadians(140)))
//                .strafeToSplineHeading(new Vector2d(54, 96), Math.toRadians(270))//avoid bumping into submersible
//                .strafeTo(new Vector2d(54, 93)); //touch bar

//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(-4,122,Math.toRadians(140)))
//                .strafeTo(new Vector2d(10, 120))
//                .strafeTo(new Vector2d(5,125));
        Actions.runBlocking(
                new ParallelAction(
            claw.closeClaw(),
            intake_angle.RotatePosition0())
            );

        waitForStart();

        Action firstTrajectory = tab1.build();
//        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
//        Action fifthTrajectory = tab5.build();
//        Action sixthTrajectory = tab6.build();
//        Action seventhTrajectory = tab7.build();
//        Action eighthTrajectory = tab8.build();
//        Action ninthTrajectory = tab9.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                claw.closeClaw(),
                                arm1.liftHighBasket(),
                                arm2.liftHighBasket(),
                                firstTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
//                                secondTrajectory,
                                thirdTrajectory,
                                arm1.waitLiftFloor(),
                                arm2.waitLiftFloor()
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                            fourthTrajectory,
                            arm1.waitLiftHighBasket(),
                            arm2.waitLiftHighBasket()
                        ),
                        claw.openClaw()
//                        fifthTrajectory,
//                        new ParallelAction(
//                                arm1.liftFloor()
//                                //arm2.LiftFloorDown()
//                        ),
//                        claw.closeClaw()
//                        new ParallelAction(
//                                arm1.liftHighBasket(),
//                                arm2.liftHighBasket(),
//                                sixthTrajectory
//                        ),
//                        claw.openClaw(),
//                        seventhTrajectory,
//                        new ParallelAction(
//                                arm1.liftDown(),
//                                arm2.liftDown(),
//                                intake_angle.RotatePosition1()
//                        ),
//                        eighthTrajectory,
//                        ninthTrajectory
                )
        );
    }
}
