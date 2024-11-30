package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@TeleOp(name = "FtcTeleOp2023")
public class FtcTeleOp2023 extends LinearOpMode {

    private DcMotor W_FR;
    private DcMotor W_FL;
    private DcMotor W_BR;
    private DcMotor W_BL;
    private DcMotor Lift;
    private DcMotor Pull_Up;
    private DcMotor Intake_Angle;
    private Servo Plane;
    private Servo Intake1;
    private Servo Intake2;
    private IMU imu;

    Orientation Direction;
    float Heading_Angle;
    double Motor_power_BR;
    double Motor_power_BL;
    int imu_rotation;
    double Motor_fwd_power;
    double Motor_power_FL;
    float Targeting_Angle;
    double Motor_side_power;
    double Motor_power_FR;
    int Intake_Ang_Status;
    double Motor_Rotation_power;
    int Intake_Angle_Pos;
    double Lift_Power;
    double Motor_Power;

    /**
     * Describe this function...
     */
    private void Initialization() {
        W_FR.setDirection(DcMotor.Direction.REVERSE);
        W_FL.setDirection(DcMotor.Direction.REVERSE);
        W_BR.setDirection(DcMotor.Direction.FORWARD);
        W_BL.setDirection(DcMotor.Direction.REVERSE);
        W_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setDirection(DcMotor.Direction.REVERSE);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setTargetPosition(0);
        Pull_Up.setDirection(DcMotor.Direction.FORWARD);
        Pull_Up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pull_Up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake_Angle.setDirection(DcMotor.Direction.REVERSE);
        Intake_Angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake_Angle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake_Angle.setTargetPosition(0);
        Intake_Angle_Pos = -2100;
        Plane.setDirection(Servo.Direction.FORWARD);
        Plane.scaleRange(0.6, 0.7);
        Intake1.setDirection(Servo.Direction.FORWARD);
        Intake2.setDirection(Servo.Direction.REVERSE);
        Intake2.scaleRange(0.3, 0.45);
        Intake1.scaleRange(0.3, 0.45);
        Intake1.setPosition(0);
        Intake2.setPosition(0);
        Motor_Power = 0.65;
        Targeting_Angle = 0;
        Lift_Power = 0.3;
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        waitForStart();
        imu.resetYaw();
    }

    /**
     * Describe this function...
     */
    private void Calculate_IMU_Rotation_Power() {
        double Angle_Difference;

        Direction = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Heading_Angle = Direction.firstAngle;
        if (Math.abs(gamepad1.right_stick_x) >= 0.01) {
            imu_rotation = 0;
            Targeting_Angle = Heading_Angle;
        } else {
            Angle_Difference = Heading_Angle - Targeting_Angle;
            if (Angle_Difference > 180) {
                Angle_Difference = Angle_Difference - 360;
            } else if (Angle_Difference < -180) {
                Angle_Difference = Angle_Difference + 360;
            }
            if (Math.abs(Angle_Difference) < 1) {
                imu_rotation = 0;
            } else if (Angle_Difference >= 1) {
                imu_rotation = (int) (Angle_Difference * 0.01 + 0.1);
            } else {
                imu_rotation = (int) (Angle_Difference * 0.01 - 0.1);
            }
        }
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double Pull_Up_Status = 0;
        double intake1toggle = 0;
        double intake2toggle = 0;

        W_FR = hardwareMap.get(DcMotor.class, "W_FR");
        W_FL = hardwareMap.get(DcMotor.class, "W_FL");
        W_BR = hardwareMap.get(DcMotor.class, "W_BR");
        W_BL = hardwareMap.get(DcMotor.class, "W_BL");
        /*Lift = hardwareMap.get(DcMotor.class, "Lift");
        Pull_Up = hardwareMap.get(DcMotor.class, "Pull_Up");
        Intake_Angle = hardwareMap.get(DcMotor.class, "Intake_Angle");
        Plane = hardwareMap.get(Servo.class, "Plane");
        Intake1 = hardwareMap.get(Servo.class, "Intake1");
        Intake2 = hardwareMap.get(Servo.class, "Intake2");*/
        imu = hardwareMap.get(IMU.class, "imu");

        // Put initialization blocks here.
        Initialization();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                Pull_Up2();
                Calculate_IMU_Rotation_Power();
                Calculate_Motor_Power();
                W_BL.setPower(Motor_power_BL);
                W_BR.setPower(Motor_power_BR);
                W_FR.setPower(Motor_power_FR);
                W_FL.setPower(Motor_power_FL);
                /*Lift_Contol();
                Intake_Control();
                Intake_Angle_Control();
                artillerycannon();*/
                if (gamepad2.dpad_right) {
                    imu.resetYaw();
                }
                /*telemetry.addData("Direction", Direction.firstAngle);
                telemetry.addData("Motor Power", Motor_Power);
                telemetry.addData("Side Power", Motor_side_power);
                telemetry.addData("FWD Power", Motor_fwd_power);
                telemetry.addData("IMU_Rotation Power", imu_rotation);
                telemetry.addData("Rotation Power", Motor_Rotation_power);
                telemetry.addData("Pull_Up_Status", Pull_Up_Status);
                telemetry.addData("Pull_Up_Encoder", Pull_Up.getCurrentPosition());
                telemetry.addData("Lift_Height", Lift.getCurrentPosition());
                telemetry.addData("Intake_Angle", Intake_Angle.getCurrentPosition());
                telemetry.addData("Intake1_Toggle", intake1toggle);
                telemetry.addData("Intake2_Toggle", intake2toggle);
                telemetry.addData("Plane Encoder", Plane.getPosition());
                telemetry.update();*/
            }
        }
    }

    /**
     * Describe this function...
     */
    private void artillerycannon() {
        if (gamepad2.a) {
            Plane.setPosition(1);
        }
        if (gamepad2.b) {
            Plane.setPosition(0);
        }
    }

    /**
     * Describe this function...
     */
    private void Calculate_Motor_Power() {
        float Motor_FWD_input;
        float Motor_Side_input;

        if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
            Motor_FWD_input = gamepad1.left_stick_y;
            Motor_Side_input = -gamepad1.left_stick_x;
            Motor_fwd_power = Math.cos(Heading_Angle / 180 * Math.PI) * Motor_FWD_input - Math.sin(Heading_Angle / 180 * Math.PI) * Motor_Side_input;
            Motor_side_power = (Math.cos(Heading_Angle / 180 * Math.PI) * Motor_Side_input + Math.sin(Heading_Angle / 180 * Math.PI) * Motor_FWD_input) * 1.5;
            Motor_Rotation_power = gamepad1.right_stick_x * 0.7 + imu_rotation;
            Motor_power_BL = -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Motor_Power);
            Motor_power_BR = -((Motor_fwd_power + Motor_side_power + Motor_Rotation_power) * Motor_Power);
            Motor_power_FL = -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Motor_Power);
            Motor_power_FR = (((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Motor_Power);
        } else {
            Motor_power_BR = 0;
            Motor_power_BL = 0;
            Motor_power_FL = 0;
            Motor_power_FR = 0;
        }
    }

    /**
     * Describe this function...
     */
    private void Intake_Control() {
        if (gamepad1.left_trigger > 0.5) {
            Intake2.setPosition(0);
        } else if (gamepad1.left_bumper) {
            Intake2.setPosition(1);
        }
        if (gamepad1.right_trigger > 0.5) {
            Intake1.setPosition(0);
        } else if (gamepad1.right_bumper) {
            Intake1.setPosition(1);
        }
    }

    /**
     * Describe this function...
     */
    private void Lift_Contol() {
        int Lift_Control = 0;

        if (gamepad1.dpad_right == true) {
            Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift_Control = 1;
        }
        if (gamepad1.y && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
            Lift.setTargetPosition(2900);
            Lift_Control = 0;
            Intake_Angle.setTargetPosition(Intake_Angle_Pos);
            Intake_Ang_Status = 0;
        } else if (gamepad1.x && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
            Lift.setTargetPosition(1500);
            Lift_Control = 0;
            Intake_Angle.setTargetPosition(Intake_Angle_Pos);
            Intake_Ang_Status = 0;
        } else if (gamepad1.a && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
            Lift.setTargetPosition(0);
            Lift_Control = 0;
            Intake_Angle.setTargetPosition(0);
            Intake_Ang_Status = 0;
        } else if (gamepad1.b && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
            Lift.setTargetPosition(800);
            Lift_Control = 0;
            Intake_Angle.setTargetPosition(Intake_Angle_Pos);
            Intake_Ang_Status = 0;
        } else if (gamepad1.right_stick_button && !(gamepad1.left_bumper || gamepad1.right_bumper)) {
            Lift.setTargetPosition(0);
            Lift_Control = 0;
            Intake_Angle.setTargetPosition(Intake_Angle_Pos / 2);
            Intake_Ang_Status = 0;
        }
        if ((gamepad1.dpad_up || gamepad1.dpad_down) && !gamepad1.start) {
            Lift_Control = 1;
        }
        if (Lift_Control > 0.5 && !gamepad1.back) {
            if (gamepad1.dpad_up) {
                Lift.setPower(1 * Lift_Power);
            } else if (gamepad1.dpad_down) {
                Lift.setPower(1 * -Lift_Power);
            }
            Lift.setTargetPosition(Lift.getCurrentPosition());
            Lift_Control = 0;
        } else {
            if (Lift.getTargetPosition() - Lift.getCurrentPosition() >= 500) {
                ((DcMotorEx) Lift).setVelocity(Math.min(Math.max(((DcMotorEx) Lift).getVelocity() + 200, 0), 1500));
            } else if (Math.abs(Lift.getTargetPosition() - Lift.getCurrentPosition()) <= 60 && Math.abs(Lift.getTargetPosition()) <= 60) {
                ((DcMotorEx) Lift).setVelocity(0);
            } else if (Lift.getTargetPosition() - Lift.getCurrentPosition() >= -500) {
                ((DcMotorEx) Lift).setVelocity((Lift.getTargetPosition() - Lift.getCurrentPosition()) * 3);
            } else {
                ((DcMotorEx) Lift).setVelocity(Math.min(Math.max(((DcMotorEx) Lift).getVelocity() - 200, -2000), 0));
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Intake_Angle_Control() {
        if (gamepad1.dpad_left == true && gamepad1.start) {
            Intake_Angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Intake_Angle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Intake_Angle.setTargetPosition(0);
            Intake_Ang_Status = 1;
        }
        if (gamepad1.dpad_right == true && gamepad1.start) {
            imu.resetYaw();
        }
        if (gamepad1.dpad_down == true && gamepad1.start) {
            Intake_Angle.setTargetPosition(Intake_Angle.getCurrentPosition() - 100);
            Intake_Ang_Status = 1;
        } else if (gamepad1.dpad_up == true && gamepad1.start) {
            Intake_Angle.setTargetPosition(Intake_Angle.getCurrentPosition() + 100);
            Intake_Ang_Status = 1;
        }
        if (Intake_Ang_Status > 0.5 && gamepad1.start) {
            if (gamepad1.dpad_down) {
                Intake_Angle.setPower(1 * 0.2);
            } else if (gamepad1.dpad_up) {
                Intake_Angle.setPower(-1 * 0.2);
            }
            Intake_Angle.setTargetPosition(Intake_Angle.getCurrentPosition());
            Intake_Ang_Status = 0;
        } else {
            if (Intake_Angle.getTargetPosition() - Intake_Angle.getCurrentPosition() >= 300) {
                ((DcMotorEx) Intake_Angle).setVelocity(Math.min(Math.max(((DcMotorEx) Intake_Angle).getVelocity() + 400, 0), 2000));
            } else if (Math.abs(Intake_Angle.getTargetPosition() - Intake_Angle.getCurrentPosition()) <= 30) {
                ((DcMotorEx) Intake_Angle).setVelocity(0);
            } else if (Intake_Angle.getTargetPosition() - Intake_Angle.getCurrentPosition() >= -300) {
                ((DcMotorEx) Intake_Angle).setVelocity((Intake_Angle.getTargetPosition() - Intake_Angle.getCurrentPosition()) * 3);
            } else {
                ((DcMotorEx) Intake_Angle).setVelocity(Math.min(Math.max(((DcMotorEx) Intake_Angle).getVelocity() - 400, -2000), 0));
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Pull_Up2() {
        Pull_Up.setPower(gamepad2.right_stick_y);
    }
}