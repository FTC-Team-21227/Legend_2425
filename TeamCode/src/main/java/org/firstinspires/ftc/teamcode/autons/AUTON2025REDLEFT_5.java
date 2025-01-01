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

@Autonomous(name = "AUTONLEFT_5")
public class AUTON2025REDLEFT_5 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);
//        CLAW_ANGLE claw_angle = new CLAW_ANGLE(hardwareMap);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(2.5, 122, Math.toRadians(140)), Math.toRadians(30));// loaded sample go to basket
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(4, 123, Math.toRadians(140)))
//                .strafeToLinearHeading(new Vector2d(12, 111), Math.toRadians(0));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(2.5, 122, Math.toRadians(140)))
                .strafeToLinearHeading(new Vector2d(9, 113.5), Math.toRadians(0)) //get 1st sample from the left side
                        .strafeTo(new Vector2d(12, 113.5)); //get 1st sample
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(10, 113.5, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(10, 85), Math.toRadians(140)) //go away from wall bec arms lifting
                .strafeTo(new Vector2d(2.5, 122)); //go to basket
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(2.5, 122, Math.toRadians(140)))
                .strafeTo(new Vector2d(12, 111))//go away from while lowering arms
                .splineToLinearHeading(new Pose2d(10, 125, Math.toRadians(0)), Math.toRadians(0)) //get 2nd sample
                .strafeTo(new Vector2d(12, 125)); //go forward to grab 2nd sample
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(10, 125, Math.toRadians(0)))
                .strafeTo(new Vector2d(10, 85)) //go away from basket/wall for space to turn
                .strafeToLinearHeading(new Vector2d(2.5, 122), Math.toRadians(140));//go to basket
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(2.5,122, Math.toRadians(140)))
                .strafeTo(new Vector2d(10, 85)) //go away from while lowering arms
                .splineToLinearHeading(new Pose2d(10, 133, Math.toRadians(0)), Math.toRadians(30)) //get 3rd sample
                .strafeTo(new Vector2d(12, 134.5)); //go forward to grab 3rd sample
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
                new SequentialAction(
                        claw.closeClaw(),
                        intake_angle.RotatePosition1()
//                        claw_angle.RotatePosition0()
                )
        );

        waitForStart();

        Action firstTrajectory = tab1.build();
//        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
//        Action eighthTrajectory = tab8.build();
//        Action ninthTrajectory = tab9.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                claw.closeClaw(),
                                arm1.waitLiftHighBasket(0, 2),
                                arm2.waitLiftHighBasket(0, 2),
                                firstTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
//                                secondTrajectory,
                                thirdTrajectory,
                                arm1.waitLiftFloor(1, 2),
                                arm2.waitLiftFloor(1, 2)
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                            fourthTrajectory,
                            arm1.waitLiftHighBasket(0, 2),
                            arm2.waitLiftHighBasket(0, 2)
                        ),
                        claw.openClaw(),
                        new ParallelAction(
                                fifthTrajectory,
                                arm1.waitLiftFloor(1, 2),
                                arm2.waitLiftFloor(1, 2)
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.waitLiftHighBasket(0, 2),
                                arm2.waitLiftHighBasket(0, 2),
                                sixthTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
                                seventhTrajectory,
                                arm1.waitLiftFloor(1, 2),
                                arm2.waitLiftFloor(1, 2)
                        )
//                        eighthTrajectory,
//                        ninthTrajectory
                )
        );
    }
}
