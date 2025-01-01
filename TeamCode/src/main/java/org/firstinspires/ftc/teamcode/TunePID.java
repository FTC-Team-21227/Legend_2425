package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
//@TeleOp
public class TunePID extends OpMode {
    private DcMotorEx ARM1; //bottom arm
    private DcMotorEx ARM2; //top arm
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    private PIDController controller2;
    //PIDF gains
    public static double p1 = 0.008, i1 = 0.001, d1 = 0.0001;
    public static double f1 = 0.000001;
    public static double p2 = 0.01, i2 = 0.0001, d2 = 0.0003;
    public static double f2 = 0;
    //ARM1, ARM2 target positions, in degrees
    public static double target1 = 0;
    public static double target2 = 0;
    //ticks to degrees conversion, very useful
    private final double ticks_in_degree_1 = 537.7*28/360; // = 41.8211111111
    private final double ticks_in_degree_2 = 145.1*28/360; // = 11.2855555556
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
    @Override
    public void init(){
        controller1 = new PIDController(p1,i1,d1);
        controller2 = new PIDController(p2,i2,d2);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        ARM1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        ARM2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        ARM1.setDirection(DcMotorEx.Direction.REVERSE);
        ARM2.setDirection(DcMotorEx.Direction.REVERSE);
    }
    @Override
    public void loop(){
        controller1.setPID(p1,i1,d1);
        int arm1Pos = ARM1.getCurrentPosition();
        double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
        double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(Math.toRadians(target1)))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
        Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation, change when equation is derived
        double power1 = pid1 + ff1;
        ARM1.setPower(power1); //set the power

        controller2.setPID(p2,i2,d2);
        int arm2Pos = ARM2.getCurrentPosition();
        double pid2 = controller2.calculate(arm2Pos, (int)(target2*ticks_in_degree_2));
        double ff2 = (m2*Math.cos(Math.toRadians(target1+target2))*x2) * f2; //feedforward calculation, change when equation is derived
        double power2 = pid2 + ff2;
        ARM2.setPower(power2);

        telemetry.addData("ARM1POS",arm1Pos);
        telemetry.addData("ARM1Target",(int)(target1*ticks_in_degree_1));
        telemetry.addData("ARM2POS",arm2Pos);
        telemetry.addData("ARM2Target",(int)(target2*ticks_in_degree_2));
        telemetry.update();
    }
}
