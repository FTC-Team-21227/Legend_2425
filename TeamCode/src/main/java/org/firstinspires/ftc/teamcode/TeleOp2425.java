package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp2425")
public class TeleOp2425 extends LinearOpMode {

    private DcMotor W_BL;
    private DcMotor W_BR;
    private DcMotor W_FR;
    private DcMotor W_FL;
    private IMU imu;
    private DcMotor ARM1; //bottom arm
    private DcMotor ARM2; //top arm
    private CRServo Hook;
    private Servo Intake_Angle;
    private Servo Claw;
    private TouchSensor ARM1Sensor;
    private TouchSensor ARM2Sensor;

    float Heading_Angle;
    double Motor_power_BR;
    Orientation Direction;
    int imu_rotation;
    double Motor_power_BL;
    float Targeting_Angle;
    double Motor_fwd_power;
    double Motor_power_FL;
    double Motor_side_power;
    double Motor_power_FR;
    double Motor_Rotation_power;
    double Motor_Power;
    boolean liftManualControl = true;
    double tim;
    boolean ARM1calibrated = false;
    boolean ARM2calibrated = false;
    boolean hanging = false;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        W_BL = hardwareMap.get(DcMotor.class, "W_BL");
        W_BR = hardwareMap.get(DcMotor.class, "W_BR");
        W_FR = hardwareMap.get(DcMotor.class, "W_FR");
        W_FL = hardwareMap.get(DcMotor.class, "W_FL");
        imu = hardwareMap.get(IMU.class, "imu");
        ARM1 = hardwareMap.get(DcMotor.class, "ARM1");
        ARM2 = hardwareMap.get(DcMotor.class, "ARM2");
        Hook = hardwareMap.get(CRServo.class, "Hook");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Intake_Angle = hardwareMap.get(Servo.class,"Intake_Angle");
        ARM1Sensor = hardwareMap.get(TouchSensor.class, "ARM1Sensor");
        ARM2Sensor = hardwareMap.get(TouchSensor.class, "ARM2Sensor");

        // Put initialization blocks here.
        Initialization();
        if (opModeIsActive()) {
            ARM1.setPower(-0.2);
            ARM2.setPower(-0.2);
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                Calculate_IMU_Rotation_Power();
                Calculate_Motor_Power();
                W_BL.setPower(Motor_power_BL);
                W_BR.setPower(Motor_power_BR);
                W_FR.setPower(Motor_power_FR);
                W_FL.setPower(Motor_power_FL);
                ARM_Control();
                Intake_Control();
                Hook_Control();
                if (gamepad1.back) {
                    imu.resetYaw();
                }
                telemetry.addData("Direction", Direction.firstAngle);
                telemetry.addData("Motor Power", Motor_Power);
                telemetry.addData("Side Power", Motor_side_power);
                telemetry.addData("FWD Power", Motor_fwd_power);
                telemetry.addData("IMU_Rotation Power", imu_rotation);
                telemetry.addData("Rotation Power", Motor_Rotation_power);
                telemetry.addData("Rotation Power", Motor_Rotation_power);
                telemetry.addData("ODO_Left", W_FL.getCurrentPosition());
                telemetry.addData("ODO_Right", W_FR.getCurrentPosition());
                telemetry.addData("ODO_Center", W_BR.getCurrentPosition());
                telemetry.addData("ARM1_Position", ARM1.getCurrentPosition());
                telemetry.addData("ARM2_TargetPosition", ARM2.getTargetPosition());
                telemetry.addData("ARM2_Position", ARM2.getCurrentPosition());
                telemetry.addData("ARM1 Pressed:",ARM1Sensor.isPressed());
                telemetry.addData("Claw", Claw.getPosition());
                telemetry.addData("Intake angle", Intake_Angle.getPosition());
                telemetry.update();
            }
        }
    }
    private void Intake_Control(){
        if (gamepad1.left_bumper) {
            Intake_Angle.setPosition(0); //forward
        }
        if (gamepad1.left_trigger > 0.1) {
            Intake_Angle.setPosition(1); //right
        }
        if (gamepad1.right_bumper) {
            Claw.setPosition(0); //close
        }
        if (gamepad1.right_trigger > 0.1) {
            Claw.setPosition(1); //open
        }
    }
    private void Hook_Control(){
        if (gamepad2.right_bumper) { //down?
            tim = getRuntime();
            Hook.setPower(0.5);
        }
        if (gamepad2.left_bumper) { //up?
            tim = getRuntime();
            Hook.setPower(-0.5);
        }
        if (getRuntime() > tim+1) {
            Hook.setPower(0);
        }
    }
    private void ARM_Control() {
//        telemetry.addData("ARM1Calibrated",ARM1calibrated);
//        telemetry.addData("ARM2Calibrated", ARM2calibrated);
//        telemetry.addData("where am i going", !(ARM1calibrated && ARM2calibrated));
        if (!(ARM1calibrated && ARM2calibrated)){
//            double a = getRuntime();
//            ARM1.setPower(0.5);
//            while (getRuntime()-a < 1){}
//            ARM1.setPower(0);
//            a = getRuntime();
//            ARM2.setPower(0.5);
//            while (getRuntime()-a < 1){}
//            ARM2.setPower(0);
            if (!ARM1Sensor.isPressed() && !ARM1calibrated) {
                ARM1.setPower(0);
                ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                ARM1.setTargetPosition(0);
                ARM1calibrated = true;
//                telemetry.addData("arm1:",ARM1Sensor.isPressed());
//                telemetry.update();
            }
            if (!ARM2Sensor.isPressed() && !ARM2calibrated) {
                ARM2.setPower(0);
                ARM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ARM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                ARM2.setTargetPosition(0);
                ARM2calibrated = true;
//                telemetry.addData("arm2:",ARM2Sensor.isPressed());
//                telemetry.update();
            }
        }
        else {
//            telemetry.addData("We Are Heree","yesd");
            if (gamepad2.a) { //prepare for hang
                ARM1.setTargetPosition(12639);
                ARM2.setTargetPosition(7976);
                hanging = true;
            }
            if (gamepad2.b) { //hang
                ARM1.setTargetPosition(-712);
                Intake_Angle.setPosition(1);
                Claw.setPosition(1);
                hanging = true;
            }
            if (gamepad1.x) {//high rung
                ARM1.setTargetPosition(4108);
                ARM2.setTargetPosition(6308);
                liftManualControl = false;
            }
            if (gamepad1.y) {//high bucket
                ARM1.setTargetPosition(10913);
                ARM2.setTargetPosition(7508);
                liftManualControl = false;
            }
            if (gamepad1.a ) {//floor
                ARM1.setTargetPosition(0);
                ARM2.setTargetPosition(6581);
                liftManualControl = false;
            }
            if (gamepad1.b ) {//wall
                ARM1.setTargetPosition(1351);
                ARM2.setTargetPosition(6293);
                liftManualControl = false;
            }
            if (gamepad1.start) { //into submersible
                ARM1.setTargetPosition(542);
                ARM2.setTargetPosition(6436);
                liftManualControl = false;
            }
            //        if (gamepad2.left_trigger > 0.3 || gamepad2.right_trigger > 0.3) {
            //            while (ARM1Sensor.isPressed()) {
            //                ARM1.setPower(-0.2);
            //            }
            //            ARM1.setPower(0);
            //            while (ARM2Sensor.isPressed()) {
            //                ARM2.setPower(-0.2);
            //            }
            //            ARM2.setPower(0);
            //        }
            if (gamepad1.right_stick_y < -0.7) {
                ARM1.setTargetPosition(ARM1.getCurrentPosition() + 250);
                liftManualControl = true;
            } else if (gamepad1.right_stick_y > 0.7) {
                ARM1.setTargetPosition(ARM1.getCurrentPosition() - 250);
                liftManualControl = true;
            }
            //SOFTWARE LIMITSSSS:
            if (ARM1.getCurrentPosition() > 12700 && !hanging) {
                ARM1.setTargetPosition(12600);
            }
            //        else if (ARM1.getCurrentPosition()+ARM2.getCurrentPosition()>8000){
            //            ARM2.setTargetPosition(ARM2.getCurrentPosition());
            //        }
            if (Math.abs(ARM1.getCurrentPosition() - ARM1.getTargetPosition()) < 15) {
                ARM1.setPower(0);
            } else {
                if (ARM1.getCurrentPosition() < ARM1.getTargetPosition())
                    ARM1.setPower(Math.abs(ARM1.getCurrentPosition() - ARM1.getTargetPosition()) * 0.01);
                else
                    ARM1.setPower(Math.abs(ARM1.getCurrentPosition() - ARM1.getTargetPosition()) * -0.01);
                ARM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.dpad_up) {
                ARM2.setTargetPosition(ARM2.getCurrentPosition() + 250);
                liftManualControl = true;
            } else if (gamepad1.dpad_down) {
                ARM2.setTargetPosition(ARM2.getCurrentPosition() - 250);
                liftManualControl = true;
            }
            if (ARM1.getCurrentPosition() > 5055 && !hanging){
                ARM2.setTargetPosition(7508);
            }
            //        if (ARM2.getCurrentPosition() > 11000 && ARM1.getCurrentPosition() > 6000){
            //            ARM2.setTargetPosition(10700);
            //        }
            if (Math.abs(ARM2.getCurrentPosition() - ARM2.getTargetPosition()) < 15) {
                ARM2.setPower(0);
            } else {
                if (ARM2.getCurrentPosition() < ARM2.getTargetPosition())
                    ARM2.setPower(Math.abs(ARM2.getCurrentPosition() - ARM2.getTargetPosition()) * 0.002);
                else
                    ARM2.setPower(Math.abs(ARM2.getCurrentPosition() - ARM2.getTargetPosition()) * -0.002);
                ARM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Initialization() {
        double Lift_Power;

        W_FR.setDirection(DcMotor.Direction.REVERSE);
        W_FL.setDirection(DcMotor.Direction.REVERSE);
        W_BR.setDirection(DcMotor.Direction.FORWARD);
        W_BL.setDirection(DcMotor.Direction.REVERSE);
        W_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        W_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake_Angle.scaleRange(0.65, 0.98);
        ARM1.setDirection(DcMotor.Direction.REVERSE);
        ARM1.setPower(0);
        ARM1.setTargetPosition(ARM1.getCurrentPosition());
        ARM2.setDirection(DcMotor.Direction.REVERSE);
        ARM2.setPower(0);
        ARM2.setTargetPosition(ARM1.getCurrentPosition());
        telemetry.addData("Claw",Claw.getPosition());
        telemetry.update();
        Claw.scaleRange(0.2,0.8);


        Motor_Power = 0.6;
        Targeting_Angle = 0;
        Lift_Power = 0.3;
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        waitForStart();
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
}
