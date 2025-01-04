package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PathBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Autonomous(name = "AUTONLEFT_4")
public class AUTON2025REDLEFT_4 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(10);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
                .strafeTo(new Vector2d(5,72));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 72, 0))
                .strafeTo(new Vector2d(8, 114.5))
                .strafeTo(new Vector2d(10.5, 114.5));//first sample
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(10.5, 114.5, 0))
                .waitSeconds(2.75)
                .turnTo(Math.toRadians(140)) //face basket
                .strafeTo(new Vector2d(0, 122));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(0, 122, Math.toRadians(140)))
                .strafeToSplineHeading(new Vector2d(12,112),Math.toRadians(0))
                //.turnTo(Math.toRadians(0))//get second sample
                .strafeTo(new Vector2d(8.25,124.5)); //change
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(8.25, 124.5, Math.toRadians(0)))
                .waitSeconds(2.8)
                .strafeToSplineHeading(new Vector2d(15,115),Math.toRadians(145)) //face basket
                .strafeTo(new Vector2d(-1.5, 122));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-1.5,122, Math.toRadians(145)))
                .strafeToSplineHeading(new Vector2d(3, 116),0);
        TrajectoryActionBuilder waitTab = drive.actionBuilder(new Pose2d(0,122, Math.toRadians(145)))
                .waitSeconds(1)
                .turn(Math.toRadians(0.1));
        TrajectoryActionBuilder waitTab2 = drive.actionBuilder(new Pose2d(0,122, Math.toRadians(145)))
                .waitSeconds(1)
                .turn(Math.toRadians(0.1));

//        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(-4,122,Math.toRadians(140)))
//                .strafeTo(new Vector2d(10, 120))
//                .strafeTo(new Vector2d(5,125));

        claw.closeClaw();
        intake_angle.RotatePosition0();

        waitForStart();

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
        Action wait = waitTab.build();
        Action wait2 = waitTab2.build();
        //Action eighthTrajectory = tab8.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                arm1.liftRung(),
                                arm2.liftRung(),
                                claw.closeClaw(),
                                firstTrajectory
                        ),
                        //arm1.hookSpecimen(),
                        secondTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.liftFloor(),
                                arm2.liftFloor(),
                                thirdTrajectory
                        ),
                        claw.closeClaw(),
                        wait,
                        new ParallelAction(
                            fourthTrajectory,
                            arm1.liftHighBasket(),
                            arm2.liftHighBasket()
                        ),
                        claw.openClaw(),
                        fifthTrajectory,
                        new ParallelAction(
                                arm1.liftFloor()
                                //arm2.LiftFloorDown()
                        ),
                        claw.closeClaw(),
                        wait2,
                        new ParallelAction(
                                arm1.liftHighBasket(),
                                arm2.liftHighBasket(),
                                sixthTrajectory
                        ),
                        claw.openClaw(),
                        seventhTrajectory,
                        new ParallelAction(
                                arm1.liftDown(),
                                arm2.liftDown(),
                                intake_angle.RotatePosition1()
                        )
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                eighthTrajectory,
//                                arm1.liftLowBasketUp(),
//                                arm2.liftLowBasketUp()
//                        ),
//                        claw.openClaw()
                        //         trajectoryActionCloseOut
                )
        );
        PoseStorage.currentPose = drive.pose;
    }
}
