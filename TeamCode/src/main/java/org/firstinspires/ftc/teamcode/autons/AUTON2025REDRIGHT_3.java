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

//@Autonomous(name = "AUTONRIGHT_3")
public class AUTON2025REDRIGHT_3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 55, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .strafeTo(new Vector2d(10,60));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 60, 0))
                .strafeToSplineHeading(new Vector2d(5,60),0)
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 60, 0))
                .strafeTo(new Vector2d(5, 30))
                .strafeTo(new Vector2d(43,30))
                .strafeTo(new Vector2d(43, 20))
                .strafeTo(new Vector2d(13,20))
                .strafeTo(new Vector2d(43,20))
                .strafeTo(new Vector2d(43,8))
                .strafeToSplineHeading(new Vector2d(13,8),0);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(13, 8, 0))
                //.strafeTo(new Vector2d(5,12))
                .strafeTo(new Vector2d(6,50))
                .turnTo(Math.toRadians(-90));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(6, 50, Math.toRadians(-90)))
                .strafeTo(new Vector2d(6,30))
                .strafeTo(new Vector2d(6,29.75));
                //.strafeTo(new Vector2d(6,30));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(6, 30, Math.toRadians(-90)))
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(5,65));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(5, 65, 0))
                .strafeTo(new Vector2d(18,65));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(18, 65, 0))
                .strafeTo(new Vector2d(0, 10));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(10, 72, 0))
                .waitSeconds(0.5);

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
        Action bbb = tab9.build();



        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                claw.closeClaw(),
                                intake_angle.RotatePosition0(),
                                arm1.liftRung(),
                                arm2.liftRung(),
                                firstTrajectory
                        ),
                        //arm1.hookSpecimen(),
                        //arm2.hookSpecimen(),
                        secondTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.liftDown(),
                                arm2.liftDown(),
                                thirdTrajectory
                        ),
                        fourthTrajectory,
                        new ParallelAction(
                            arm1.liftWall(),
                            arm2.liftWall()
                        ),
                        fifthTrajectory,
                        claw.closeClaw(),
                        bbb,
                        new ParallelAction(
                            arm1.liftRung(),
                            arm2.liftRung(),
                            sixthTrajectory
                        ),
                        seventhTrajectory,
                        //arm1.hookSpecimen(),
                        new ParallelAction(
                            claw.openClaw(),
                            eighthTrajectory,
                            arm1.liftDown(),
                            arm2.liftDown(),
                            intake_angle.RotatePosition1()
                        )
                )
        );
    }
}
