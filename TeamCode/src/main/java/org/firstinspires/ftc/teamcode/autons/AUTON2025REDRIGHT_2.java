package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

//@Autonomous(name = "AUTON2025REDRIGHT_2")
public class AUTON2025REDRIGHT_2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 53.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //waitForStart();
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
            .lineToX(20)
                .waitSeconds(1)
                .lineToX(17)
                .strafeTo(new Vector2d(17, 16))
                .waitSeconds(1)
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(48, 48, Math.toRadians(270)), Math.toRadians(270))
                .splineTo(new Vector2d(15, 6), Math.toRadians(270));
//                .turn(Math.toRadians(-180));

//            .splineTo(new Vector2d(10, 35), -Math.PI/2 )
//            .waitSeconds(1)
//            .strafeTo(new Vector2d(35,12))
//            .waitSeconds(1)
//            .strafeTo(new Vector2d(5, 12))
//                .waitSeconds(1)
//                .setTangent(0)
//                .waitSeconds(1);
        Action trajectoryActionCloseOut = tab1.fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        //if (isStopRequested()) return;

        Action trajectoryActionChosen;
        //if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        //}
//        } else if (startPosition == 2) {
//            trajectoryActionChosen = tab2.build();
//        } else {
//            trajectoryActionChosen = tab3.build();
//        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
//                        lift.liftUp(),
//                        claw.openClaw(),
//                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
