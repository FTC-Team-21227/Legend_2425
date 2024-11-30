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

//@Autonomous(name = "AutonArmServo")
public class AutonArmServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //waitForStart();
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(-10, -10));
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(0, 0, Math.toRadians(-180)), Math.toRadians(-270))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(0))
//                .waitSeconds(2)
//                .splineToSplineHeading(new Pose2d(22, 22, Math.toRadians(180)), Math.toRadians(90));
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
