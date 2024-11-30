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

//@Autonomous(name = "AUTON2025REDLEFT")
public class AUTON2025REDLEFT extends LinearOpMode {
    public class ARM1 {
        private DcMotorEx arm1;

        public ARM1(HardwareMap hardwareMap) {
            arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
            arm1.setDirection(DcMotorSimple.Direction.REVERSE);
            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm1.setPower(0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//                packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 5500) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm1.setPower(-0.8);
                    initialized = true;
                }

                double pos = arm1.getCurrentPosition();
                telemetry.addData("liftPos", pos);
                //packet.put("liftPos", pos);
                telemetry.update();
                if (pos > 200) {
                    return true;
                } else {
                    arm1.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    public class ARM2 {
        private DcMotorEx arm2;

        public ARM2(HardwareMap hardwareMap) {
            arm2 = hardwareMap.get(DcMotorEx.class, "ARM2"); //ARM2
            arm2.setDirection(DcMotorSimple.Direction.REVERSE);
            arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }
        //public

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm2.setPower(0.8);
                    initialized = true;
                }

                double pos = arm2.getCurrentPosition();
                telemetry.addData("liftPos", pos);
//              packet.put("liftPos", pos);
                telemetry.update();
                if (pos < 8000) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm2.setPower(-0.8);
                    initialized = true;
                }

                double pos = arm2.getCurrentPosition();
                telemetry.addData("liftPos", pos);
                //packet.put("liftPos", pos);
                telemetry.update();
                if (pos > 200) {
                    return true;
                } else {
                    arm2.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1 arm1 = new ARM1(hardwareMap);
        ARM2 arm2 = new ARM2(hardwareMap);

        //waitForStart();
        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(20)
                .waitSeconds(1)
                .strafeTo(new Vector2d(10, 90))
                .waitSeconds(1)
                .strafeTo(new Vector2d(35,110))
                .waitSeconds(1)
                .strafeTo(new Vector2d(10, 120));
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
             //           trajectoryActionChosen,
                        arm1.liftUp(),
                        arm2.liftUp(),
//                        claw.openClaw(),
                        arm2.liftDown(),
                        arm1.liftDown()
               //         trajectoryActionCloseOut
                )
        );
    }
}
