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

//@Autonomous(name = "AUTONRIGHT_4")
public class AUTON2025REDRIGHT_4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(15, -63.3, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(1)
                .strafeTo(new Vector2d(10,-41.3));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, -41.3, Math.toRadians(90)))
                .strafeTo(new Vector2d(10,-53));
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(10, -53, Math.toRadians(90)))
                .strafeTo(new Vector2d(40,-53))
                .strafeTo(new Vector2d(40, -20))
                .strafeTo(new Vector2d(50, -20))
                .strafeTo(new Vector2d(50, -53))
                .strafeTo(new Vector2d(50, -20))
                .strafeTo(new Vector2d(62, -20))
                .strafeTo(new Vector2d(62, -53));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(62, -53, Math.toRadians(90)))
                //.strafeTo(new Vector2d(5,12))
                .strafeToSplineHeading(new Vector2d(20,-55),Math.toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(20, -55, Math.toRadians(0)))
                .strafeTo(new Vector2d(40,-55));
                //.strafeTo(new Vector2d(6,30));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(40, -55, Math.toRadians(0)))
                .waitSeconds(1)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(22,-55))
                .strafeTo(new Vector2d(5,-55));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(5, -55, Math.toRadians(90)))
                .strafeTo(new Vector2d(5,-41.3));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(5, -41.3, Math.toRadians(90)))
                .strafeTo(new Vector2d(5,-55))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(40,-55));
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
                        new ParallelAction(
                            arm1.liftRung(),
                            arm2.liftRung()
                        ),
                        sixthTrajectory,
                        seventhTrajectory,
                        new ParallelAction(
                            claw.openClaw(),
                            eighthTrajectory,
                            arm1.liftDown(),
                            arm2.liftDown(),
                            intake_angle.RotatePosition1()
                        )
//                    new ParallelAction(
//                            claw.closeClaw(),
//                            intake_angle.RotatePosition0(),
//                            arm1.liftRung(),
//                            arm2.liftRung()
//                    ),
//                    claw.openClaw(),
//                    new ParallelAction(
//                            arm1.liftDown(),
//                            arm2.liftDown()
//                    ),
//                    new ParallelAction(
//                            arm1.liftWall(),
//                            arm2.liftWall()
//                    ),
//                    claw.closeClaw(),
//                    new ParallelAction(
//                            arm1.liftRung(),
//                            arm2.liftRung()
//                    ),
//                    new ParallelAction(
//                            claw.openClaw(),
//                            arm1.liftDown(),
//                            arm2.liftDown(),
//                            intake_angle.RotatePosition1()
//                    )
                )
        );
    }
}
