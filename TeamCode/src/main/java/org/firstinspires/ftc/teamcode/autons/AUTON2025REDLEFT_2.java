package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

//@Autonomous(name = "AUTON2025REDLEFT_2")
public class AUTON2025REDLEFT_2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //waitForStart();
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(20)
                .waitSeconds(1)
                .lineToX(17)
                .strafeTo(new Vector2d(17, 110))
                .waitSeconds(1)
                .turnTo(-Math.PI/4)
                .waitSeconds(1)
                //.setTangent(3*Math.PI/4)
                .strafeTo(new Vector2d(11,112))
                .waitSeconds(1)
                .strafeTo(new Vector2d(17,110))
                .waitSeconds(1)
                .turnTo(Math.PI/6)
                .waitSeconds(1)
                .turnTo(-Math.PI/4)
                .strafeTo(new Vector2d(11,112))
                .waitSeconds(1)
                .strafeTo(new Vector2d(17, 110))
                .waitSeconds(1)
                .turnTo(Math.PI/4)
                .waitSeconds(1)
                .turnTo(-Math.PI/4)
                .strafeTo(new Vector2d(11,112));
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)

//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.


//        while (!isStopRequested() && !opModeIsActive()) {
//            int position = visionOutputPosition;
//            telemetry.addData("Position during Init", arm1.getCurrentPosition);
//            telemetry.update();
//        }
//
//        int startPosition = visionOutputPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
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
                        trajectoryActionChosen
                        //         trajectoryActionCloseOut
                )
        );
    }
}
