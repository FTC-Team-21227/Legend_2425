package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
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
import org.firstinspires.ftc.teamcode.TunePID;
import org.firstinspires.ftc.teamcode.autons.PoseStorage;

//@Config
//@TeleOp(name = "TeleOp2425_NoCal")
//Most recent teleop for the old robot with no calibrations
public class TeleOp2425_NoCal extends LinearOpMode {
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    private PIDController controller2;
    //PIDF gains
    double p1 = TunePID.p1, i1 = TunePID.i1, d1 = TunePID.d1;
    double f1 = TunePID.f1;
    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
    double f2 = TunePID.f2;
    //ARM1, ARM2 target positions, in degrees
    double target1 = 0;
    double target2 = 0;
    //ticks to degrees conversion, very useful
    private final double ticks_in_degree_1 = 537.7*28/360; // = 41.8211111111
    private final double ticks_in_degree_2 = 145.1*28/360; // = 11.2855555556
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
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

    double Heading_Angle;
    double Motor_power_BR;
    Orientation Direction;
    int imu_rotation;
    double Motor_power_BL;
    double Targeting_Angle;
    double Motor_fwd_power;
    double Motor_power_FL;
    double Motor_side_power;
    double Motor_power_FR;
    double Motor_Rotation_power;
    double Motor_Power;
    double tim;
    boolean ARM1calibrated = true;
    boolean ARM2calibrated = true;
    boolean hanging = false;
    int arm1Pos;
    int arm2Pos;
    double initialHeading;
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException{
        //getting all the motors, servos, and sensors from the hardware map
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d initialPose;
        try {
            initialPose = PoseStorage.currentPose;
            telemetry.addData("Yay1!","Yay1!");
        }
        catch (Exception e){
            initialPose = new Pose2d(0,0,0);
            telemetry.addData("No!","No!");
        }
        telemetry.update();
        initialHeading = Math.toDegrees(initialPose.heading.toDouble());
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
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                Calculate_IMU_Rotation_Power(); //calculates each motor power based on IMU reading
                Calculate_Motor_Power(); //calculates translational and rotational motor power
                //set power to each wheel motor
                W_BL.setPower(Motor_power_BL);
                W_BR.setPower(Motor_power_BR);
                W_FR.setPower(Motor_power_FR);
                W_FL.setPower(Motor_power_FL);
                //controls the arm motor powers
                if (!(ARM1calibrated && ARM2calibrated)) {
                    ARM_Calibration(); //calibration function
                }
                else {
                    ARM_SetTargets(); //gamepad presets and other things to set the target positions for each arm motor
                    ARM_PID_Control(); //PID control function based on target positions
                }
                //controls the intake servos
                Intake_Control();
                //controls the hook servo
                Hook_Control();
                //reset imu if necessary
                if (gamepad1.back) {
//                    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
                    imu.resetYaw();
                    Heading_Angle = 0;
                    Targeting_Angle = 0;
                }
                //        telemetry.addData("ARM1Calibrated",ARM1calibrated);
                //        telemetry.addData("ARM2Calibrated", ARM2calibrated);
                //        telemetry.addData("where am i going", !(ARM1calibrated && ARM2calibrated));
                telemetry.addData("Direction", Heading_Angle);
                telemetry.addData("Motor Power", Motor_Power);
                telemetry.addData("Side Power", Motor_side_power);
                telemetry.addData("FWD Power", Motor_fwd_power);
                telemetry.addData("IMU_Rotation Power", imu_rotation);
                telemetry.addData("Rotation Power", Motor_Rotation_power);
                telemetry.addData("ODO_Left", W_FL.getCurrentPosition());
                telemetry.addData("ODO_Right", W_FR.getCurrentPosition());
                telemetry.addData("ODO_Center", W_BR.getCurrentPosition());
                telemetry.addData("ARM1Pos: ", arm1Pos/ticks_in_degree_1);
                telemetry.addData("ARM1Target: ", target1);
                telemetry.addData("ARM2 Current Angle: ", arm2Pos/ticks_in_degree_2);
                telemetry.addData("ARM2 Target Angle: ", target2);
                telemetry.addData("ARM1 Pressed:",ARM1Sensor.isPressed());
                telemetry.addData("ARM2 Pressed:",ARM2Sensor.isPressed());
                telemetry.addData("Claw", Claw.getPosition());
                telemetry.addData("Intake angle", Intake_Angle.getPosition());
                telemetry.addData("ARM1 Power", ARM1.getPower());
                telemetry.addData("ARM2 Power", ARM2.getPower());
                telemetry.addData("Initial heading: ",initialHeading);


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
        if (getRuntime() > tim+1.5) {
            Hook.setPower(0);
        }
    }
    private void ARM_PID_Control(){
        controller1.setPID(p1,i1,d1);
        arm1Pos = ARM1.getCurrentPosition();
        double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
        double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(Math.toRadians(target1)))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
        Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation, change when equation is derived
        double power1 = pid1 + ff1;
        //telemetry.addData("ff1",ff1);
        ARM1.setPower(power1); //set the power

        controller2.setPID(p2,i2,d2);
        arm2Pos = ARM2.getCurrentPosition();
        double pid2 = controller2.calculate(arm2Pos, (int)(target2*ticks_in_degree_2));
        double ff2 = (m2*Math.cos(Math.toRadians(target1+target2))*x2) * f2; //feedforward calculation, change when equation is derived
        double power2 = pid2 + ff2;
        ARM2.setPower(power2);
    }
    private void ARM_Calibration(){
        //resets each motor to 0 when the touch sensor is pressed, and doesn't enter the if statement afterwards
        if (!ARM1Sensor.isPressed() && !ARM1calibrated) {
            ARM1.setPower(0);
            ARM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ARM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target1 = 0;
            ARM1calibrated = true;
        }
//        else if (!ARM1calibrated){
//            ARM1.setPower(-0.2);
//        }
        if (!ARM2Sensor.isPressed() && !ARM2calibrated) {
            ARM2.setPower(0);
            ARM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ARM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ARM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target2 = 0;
            ARM2calibrated = true;
        }
//        else if (!ARM2calibrated){
//            ARM2.setPower(-0.2);
//        }
    }
    private void ARM_SetTargets() {
//            telemetry.addData("We Are Heree","yesd");
        if (gamepad2.start){ //recalibrate the motors
            ARM1calibrated = false;
            ARM2calibrated = false;
            ARM1.setPower(-0.2);
            ARM2.setPower(-0.2);
        }
        if (gamepad2.a) { //prepare for hang
            target1 = 113.330920056;
            target2 = 191.742752;
            hanging = true;
        }
        if (gamepad2.b) { //hang
            target1 = -6.38433539679; //might need to change this, doesn't make sense to lower it past 0.
            //ARM2 must be moved up manually.
            Intake_Angle.setPosition(1); //setting servos in here is bad practice, but it's fine
            Claw.setPosition(1);
            hanging = true;
        }
        if (gamepad1.x) {//high rung
            target1 = 3.4193;
            target2 = 95.3431;
        }
        if (gamepad1.y) {//high bucket
            target1 = 90.5763; //97.854286777
            target2 = 169.4201; //180.492048747
        }
        if (gamepad1.a ) {//floor
            target1 = 2.6303;
            target2 = 164.6352;
        }
        if (gamepad1.b ) {//wall
            target1 = 12.0513;
            target2 = 154.1794; //155.7743
        }
        if (gamepad1.start) { //into submersible
            target1 = 4.85998565318;
            target2 = 154.721207477;
        }
        if (gamepad1.right_stick_button){ //retract both arms
            target1 = 4.48338159887;
            target2 = 5.0199819357;
        }
        if (gamepad1.right_stick_y < -0.7) { //manual control of ARM1
            target1 += 0.89667631977;
        } else if (gamepad1.right_stick_y > 0.7) {
            target1 -= 0.89667631977;
        }
        if (gamepad1.dpad_up) { //manual control of ARM2
            target2 += 2.40399638714/3;
        } else if (gamepad1.dpad_down) {
            target2 -= 2.40399638714/3;
        }
        //SOFTWARE LIMITSSSS:
//        if (ARM1.getCurrentPosition() > 12700 && !hanging) {
//            ARM1.setTargetPosition(12600);
//        }
//        else if (ARM1.getCurrentPosition()+ARM2.getCurrentPosition()>8000){
//            ARM2.setTargetPosition(ARM2.getCurrentPosition());
//        }
//        if (ARM1.getCurrentPosition() > 5055 && !hanging){
//            ARM2.setTargetPosition(7508);
//        }
//        if (ARM2.getCurrentPosition() > 11000 && ARM1.getCurrentPosition() > 6000){
//            ARM2.setTargetPosition(10700);
//        }
    }

    /**
     * Describe this function...
     */
    private void Initialization() {
        double Lift_Power;

        controller1 = new PIDController(p1, i1, d1);
        controller2 = new PIDController(p2, i2, d2);

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
        ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM1.setPower(0);
        target1 = ARM1.getCurrentPosition()/ticks_in_degree_1;
        ARM2.setDirection(DcMotor.Direction.REVERSE);
        ARM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ARM2.setPower(0);
        target2 = ARM2.getCurrentPosition()/ticks_in_degree_2;
        telemetry.addData("Claw",Claw.getPosition());
//        telemetry.update();
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
        Heading_Angle = Direction.firstAngle+initialHeading;
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
        double Motor_FWD_input;
        double Motor_Side_input;
        double mag;

        if (!(gamepad1.right_trigger>0.1 || gamepad1.right_bumper)) {
            mag = Math.sqrt(gamepad1.left_stick_y*gamepad1.left_stick_y + gamepad1.left_stick_x*gamepad1.left_stick_x);
            Motor_FWD_input = gamepad1.left_stick_y * mag;
            Motor_Side_input = -gamepad1.left_stick_x * mag;
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
