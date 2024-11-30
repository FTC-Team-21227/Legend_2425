package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//@Autonomous(name = "AUTON2025REDLEFT_3")
public class AUTON2025REDLEFT_3 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1 arm1 = new ARM1(hardwareMap);
        ARM2 arm2 = new ARM2(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(10);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
                .strafeTo(new Vector2d(5,72));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 72, 0))
                .strafeTo(new Vector2d(10, 115.5))
                .strafeTo(new Vector2d(12, 115.5))//first sample
                .waitSeconds(0.1);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(12, 115.5, 0))
                .waitSeconds(2.75)
                .turnTo(Math.toRadians(140)) //face basket
                .strafeTo(new Vector2d(-3, 122));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-3, 122, Math.toRadians(140)))
                .strafeTo(new Vector2d(9,125.5))
                .turnTo(Math.toRadians(0));//get second sample
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(9, 125.5, Math.toRadians(0)))
                .waitSeconds(2.8)
                .turnTo(Math.toRadians(145)) //face basket
                .strafeTo(new Vector2d(-3, 122));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-3,122, Math.toRadians(145)))
                .turnTo(Math.toRadians(20))//get third sample
                .strafeTo(new Vector2d(7,126.5));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(7,126.5,Math.toRadians(20)))
                .strafeTo(new Vector2d(9, 103))
                .turnTo(Math.toRadians(135)); //face basket, low basket
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
        Action eighthTrajectory = tab8.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                arm1.liftRungUp(),
                                arm2.LiftRungUp(),
                                claw.closeClaw(),
                                firstTrajectory
                        ),
                        //arm1.hookSpecimen(),
                        secondTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.LiftFloorDown(),
                                arm2.liftFloorUp(),
                                thirdTrajectory
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftBucketUp(),
                            arm2.liftBucketUp(),
                                fourthTrajectory
                        ),
                        claw.openClaw(),
                        fifthTrajectory,
                        new ParallelAction(
                                arm1.LiftFloorDown()
                                //arm2.LiftFloorDown()
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftBucketUp(),
                                //arm2.liftBucketUp(),
                                sixthTrajectory
                        ),
                        claw.openClaw(),
                        seventhTrajectory,
                        new ParallelAction(
                                arm1.LiftFloorDown()
                                //arm2.LiftFloorDown()
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                eighthTrajectory,
                                arm1.liftLowBasketUp(),
                                arm2.liftLowBasketUp()
                        ),
                        claw.openClaw()
                        //         trajectoryActionCloseOut
                )
        );
    }
}
//package org.firstinspires.ftc.teamcode.autons;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "AUTON2025REDLEFT_4")
//public class AUTON2025REDLEFT_4 extends LinearOpMode{
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        ARM1 arm1 = new ARM1(hardwareMap);
//        ARM2 arm2 = new ARM2(hardwareMap);
//        CLAW claw = new CLAW(hardwareMap);
//        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);
//
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToX(10);
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
//                .strafeTo(new Vector2d(5,72));
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 72, 0))
//                .strafeTo(new Vector2d(8, 115.5))
//                .strafeTo(new Vector2d(12, 115.5))//first sample
//                .waitSeconds(0.1);
//        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(12, 115.5, 0))
//                .waitSeconds(2.75)
//                .turnTo(Math.toRadians(140)) //face basket
//                .strafeTo(new Vector2d(0, 122));
//        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(0, 122, Math.toRadians(140)))
//                .strafeTo(new Vector2d(8,119))
//                .turnTo(Math.toRadians(0));//get second sample
//        TrajectoryActionBuilder tabb = drive.actionBuilder(new Pose2d(8, 119, Math.toRadians(140)))
//                .strafeTo(new Vector2d(8,126));
//        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(8, 126, Math.toRadians(0)))
//                .waitSeconds(2.8)
//                .turnTo(Math.toRadians(145)) //face basket
//                .strafeTo(new Vector2d(0, 122));
//        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(0,122, Math.toRadians(145)))
//                .strafeTo(new Vector2d(3, 120))
//                .turnTo(Math.toRadians(0));//get third sample;
////        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(-4,122,Math.toRadians(140)))
////                .strafeTo(new Vector2d(10, 120))
////                .strafeTo(new Vector2d(5,125));
//
//        claw.closeClaw();
//        intake_angle.RotatePosition0();
//
//        waitForStart();
//
//        Action firstTrajectory = tab1.build();
//        Action secondTrajectory = tab2.build();
//        Action thirdTrajectory = tab3.build();
//        Action fourthTrajectory = tab4.build();
//        Action fifthTrajectory = tab5.build();
//        Action sixthTrajectory = tab6.build();
//        Action seventhTrajectory = tab7.build();
//        Action bobthTrajectory = tabb.build();
//        //Action eighthTrajectory = tab8.build();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                intake_angle.RotatePosition0(),
//                                arm1.liftRungUp(),
//                                arm2.liftRungUp(),
//                                claw.closeClaw(),
//                                firstTrajectory
//                        ),
//                        //arm1.hookSpecimen(),
//                        secondTrajectory,
//                        claw.openClaw(),
//                        new ParallelAction(
//                                arm1.LiftFloorDown(),
//                                arm2.liftFloorUp(),
//                                thirdTrajectory
//                        ),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftBucketUp(),
//                                arm2.liftBucketUp(),
//                                fourthTrajectory
//                        ),
//                        claw.openClaw(),
//                        intake_angle.RotatePosition1(),
//                        new ParallelAction(
//                                fifthTrajectory,
//                                arm1.LiftFloorDown()
//                                //arm2.LiftFloorDown()
//                        ),
//                        bobthTrajectory,
//                        intake_angle.RotatePosition0(),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftBucketUp(),
//                                //arm2.liftBucketUp(),
//                                sixthTrajectory
//                        ),
//                        claw.openClaw(),
//                        seventhTrajectory,
//                        new ParallelAction(
//                                arm1.liftDown(),
//                                arm2.liftDown()
//                        )
////                        claw.closeClaw(),
////                        new ParallelAction(
////                                eighthTrajectory,
////                                arm1.liftLowBasketUp(),
////                                arm2.liftLowBasketUp()
////                        ),
////                        claw.openClaw()
//                        //         trajectoryActionCloseOut
//                )
//        );
//    }
//}
