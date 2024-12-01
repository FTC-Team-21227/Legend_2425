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

@Autonomous(name = "AUTONRIGHT_4")
public class AUTON2025REDRIGHT_4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 55, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1_NEW arm1 = new ARM1_NEW(hardwareMap);
        ARM2_NEW arm2 = new ARM2_NEW(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose) //first specimen
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(10,60),0);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 60, 0)) //pull first specimen
                .splineToConstantHeading(new Vector2d(5,60),Math.toRadians(180))
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 60, 0)) //push colored samples
                .splineToConstantHeading(new Vector2d(43, 30),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,17),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(43, 17),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,5),Math.toRadians(180));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(13, 5, 0)) //face the wall for second specimen
                .strafeToSplineHeading(new Vector2d(6,50),Math.toRadians(-90));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(6, 50, Math.toRadians(-90))) //go to second specimen
                .splineToConstantHeading(new Vector2d(6,30),Math.toRadians(-90));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(6, 30, Math.toRadians(-90))) //pick up and place second specimen
                .waitSeconds(1)
                .lineToY(35)
                .splineToSplineHeading(new Pose2d(18,65,Math.toRadians(0)),Math.toRadians(0));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(18, 65, 0)) //go to third specimen
                .lineToX(13);
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(13, 65, 0)) //go to third specimen
                .splineToSplineHeading(new Pose2d(6,30,Math.toRadians(-90)),Math.toRadians(-90));
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(6, 30, Math.toRadians(-90))) //pick up and place third specimen
                .waitSeconds(1)
                .lineToY(35)
                .splineToSplineHeading(new Pose2d(18,65,Math.toRadians(0)),Math.toRadians(0));
        TrajectoryActionBuilder tab10 = drive.actionBuilder(new Pose2d(13, 65, Math.toRadians(0)))
                .lineToX(13);
        TrajectoryActionBuilder tab11 = drive.actionBuilder(new Pose2d(18, 65, 0)) //park
                .splineToConstantHeading(new Vector2d(6,30),Math.toRadians(-90));
        TrajectoryActionBuilder waitTab = drive.actionBuilder(initialPose)
                .waitSeconds(1);

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
        Action tenthTrajectory = tab10.build();
        Action eleventhTrajectory = tab11.build();
        Action waitSecond = waitTab.build();



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
                        secondTrajectory,
                        claw.openClaw(),
                        //push colored samples
                        new ParallelAction(
                                arm1.liftDown(),
                                arm2.liftDown(),
                                thirdTrajectory
                        ),
                        //face the wall for second specimen
                        fourthTrajectory,
                        new ParallelAction(
                            arm1.liftWall(),
                            arm2.liftWall()
                        ),
                        //go to second specimen
                        fifthTrajectory,
                        claw.closeClaw(),
                        waitSecond,
                        //pick up and place second specimen
                        new ParallelAction(
                            arm1.liftRung(),
                            arm2.liftRung(),
                            sixthTrajectory
                        ),
                        seventhTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                            eighthTrajectory,
                            arm1.liftWall(),
                            arm2.liftWall()
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                            ninthTrajectory,
                            arm1.liftRung(),
                            arm2.liftRung()
                        ),
                        tenthTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                            eleventhTrajectory,
                            arm1.liftDown(),
                            arm2.liftDown(),
                            intake_angle.RotatePosition1()
                        )
                )
        );
    }
}
