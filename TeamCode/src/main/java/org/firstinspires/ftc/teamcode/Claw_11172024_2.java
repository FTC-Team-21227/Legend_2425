package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@TeleOp(name = "Claw_11172024_2 (Blocks to Java)")
public class Claw_11172024_2 extends LinearOpMode {

  private CRServo Hook;
  private DcMotor W_BL;
  private DcMotor W_BR;
  private DcMotor W_FR;
  private DcMotor W_FL;
  private IMU imu_IMU;
  private DcMotor ARM1;
  private DcMotor ARM2;
  private Servo Intake_Angle;
  private Servo Claw;
  private Servo Claw_Angle;

  Orientation Direction;
  float Heading_Angle;
  int Motor_power_BR;
  int Motor_power_BL;
  int imu_rotation;
  double Motor_fwd_power;
  int Motor_power_FL;
  float Targeting_Angle;
  double Motor_side_power;
  int Motor_power_FR;
  double Motor_Rotation_power;
  double Motor_Power;

  /**
   * Describe this function...
   */
  private void Hanging() {
    if (gamepad2.dpad_up) {
      Hook.setDirection(CRServo.Direction.FORWARD);
      Hook.setPower(0.6);
    }
    if (gamepad2.dpad_down) {
      Hook.setDirection(CRServo.Direction.REVERSE);
      Hook.setPower(0.6);
    }
    if (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
      Hook.setPower(0);
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    Hook = hardwareMap.get(CRServo.class, "Hook");
    W_BL = hardwareMap.get(DcMotor.class, "W_BL");
    W_BR = hardwareMap.get(DcMotor.class, "W_BR");
    W_FR = hardwareMap.get(DcMotor.class, "W_FR");
    W_FL = hardwareMap.get(DcMotor.class, "W_FL");
    imu_IMU = hardwareMap.get(IMU.class, "imu");
    ARM1 = hardwareMap.get(DcMotor.class, "ARM1");
    ARM2 = hardwareMap.get(DcMotor.class, "ARM2");
    Intake_Angle = hardwareMap.get(Servo.class, "Intake_Angle");
    Claw = hardwareMap.get(Servo.class, "Claw");
    Claw_Angle = hardwareMap.get(Servo.class, "Claw_Angle");

    // Put initialization blocks here.
    Initialization();
    if (opModeIsActive()) {
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
        Hanging();
        Intake_Control();
        if (gamepad1.dpad_right) {
          imu_IMU.resetYaw();
          Targeting_Angle = 0;
          Heading_Angle = 0;
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
        telemetry.addData("ARM2_Position", ARM2.getCurrentPosition());
        telemetry.addData("ARM1_Power", ARM1.getPower());
        telemetry.addData("ARM2_Power", ARM2.getPower());
        telemetry.addData("Touch Sensor", ARM2.getCurrentPosition());
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void ARM_Control() {
    if (gamepad1.dpad_up) {
      ARM1.setTargetPosition(ARM1.getCurrentPosition() + 100);
    } else if (gamepad1.dpad_down) {
      ARM1.setTargetPosition(ARM1.getCurrentPosition() - 100);
    }
    if (Math.abs(ARM1.getCurrentPosition() - ARM1.getTargetPosition()) < 15) {
      ARM1.setPower(0);
    } else {
      ARM1.setPower(Math.abs(ARM1.getCurrentPosition() - ARM1.getTargetPosition()) * 0.002);
      ARM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    if (gamepad1.y) {
      ARM2.setTargetPosition(ARM2.getCurrentPosition() + 150);
    } else if (gamepad1.a) {
      ARM2.setTargetPosition(ARM2.getCurrentPosition() - 150);
    }
    if (Math.abs(ARM2.getCurrentPosition() - ARM2.getTargetPosition()) < 15) {
      ARM2.setPower(0);
    } else {
      ARM2.setPower(Math.abs(ARM2.getCurrentPosition() - ARM2.getTargetPosition()) * 0.001);
      ARM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
  }

  /**
   * Describe this function...
   */
  private void Calculate_IMU_Rotation_Power() {
    double Angle_Difference;

    Direction = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
    ARM1.setDirection(DcMotor.Direction.REVERSE);
    ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    ARM1.setTargetPosition(0);
    ARM2.setDirection(DcMotor.Direction.REVERSE);
    ARM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ARM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    ARM2.setTargetPosition(0);
    Intake_Angle.scaleRange(0.3, 0.7);
    Intake_Angle.setPosition(1);
    Claw.scaleRange(0.2, 0.775);
    Claw_Angle.scaleRange(0.04, 0.7);
    Claw_Angle.setPosition(0.5);
    Heading_Angle = 0;
    Targeting_Angle = 0;
    Motor_Power = 0.6;
    Lift_Power = 0.3;
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    imu_IMU.resetYaw();
    waitForStart();
  }

  /**
   * Describe this function...
   */
  private void Intake_Control() {
    if (gamepad1.left_trigger > 0.5 && gamepad1.left_bumper) {
      Intake_Angle.setPosition(1);
    }
    if (gamepad1.right_trigger > 0.5 && gamepad1.right_bumper) {
      Intake_Angle.setPosition(0);
    }
    if (gamepad1.left_trigger < 0.5 && gamepad1.left_bumper) {
      Claw.setPosition(0);
    }
    if (gamepad1.right_trigger < 0.5 && gamepad1.right_bumper) {
      Claw.setPosition(1);
    }
    if (gamepad1.left_trigger > 0.5 && !gamepad1.left_bumper) {
      Claw_Angle.setPosition(1);
    }
    if (gamepad1.right_trigger > 0.5 && !gamepad1.right_bumper) {
      Claw_Angle.setPosition(0);
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
      Motor_power_BL = (int) -(((Motor_fwd_power - Motor_side_power) - Motor_Rotation_power) * Motor_Power);
      Motor_power_BR = (int) -((Motor_fwd_power + Motor_side_power + Motor_Rotation_power) * Motor_Power);
      Motor_power_FL = (int) -(((Motor_fwd_power + Motor_side_power) - Motor_Rotation_power) * Motor_Power);
      Motor_power_FR = (int) (((Motor_fwd_power - Motor_side_power) + Motor_Rotation_power) * Motor_Power);
    } else {
      Motor_power_BR = 0;
      Motor_power_BL = 0;
      Motor_power_FL = 0;
      Motor_power_FR = 0;
    }
  }
}
