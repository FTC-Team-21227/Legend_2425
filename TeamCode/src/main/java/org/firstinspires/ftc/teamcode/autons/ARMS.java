package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TunePID;

public class ARMS {
    private DcMotor arm1;
    private DcMotor arm2;
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    private PIDController controller2;
    //PIDF gains
    double p1 = TunePID.p1, i1 = TunePID.i1, d1 = TunePID.d1;
    double f1 = TunePID.f1;
    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
    double f2 = TunePID.f2;
    //ticks to degrees conversion
    private final double ticks_in_degree_1 = 41.8211111111;
    private final double ticks_in_degree_2 = 11.2855555556;
    //length, COM, mass values for feedforward calculation (not even performed in arm2)
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;

    public ARMS(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(DcMotor.class, "ARM1");
        arm1.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller1 = new PIDController(p1, i1, d1);

        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
        arm2.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller2 = new PIDController(p2, i2, d2);
    }
    public class waitLiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        boolean startMove;
        double target1;
        double target2;
        double waitTime;
        double runTime;
        public waitLiftTarget(double pos1, double pos2) {
            target1 = pos1;
            target2 = pos2;
            start = false;
            startMove = false;
            waitTime = 0;
            runTime = 2;
        }
        public waitLiftTarget(double pos1, double pos2, double tim) {
            target1 = pos1;
            target2 = pos2;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = 2;
        }
        public waitLiftTarget(double pos1, double pos2, double tim, double runtime) {
            target1 = pos1;
            target2 = pos2;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
        }
        public double ARM_Control_PID1(){
            int arm1Pos = arm1.getCurrentPosition();
            double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
            double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
                    /*NEW*/        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(Math.toRadians(target1)))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
                    Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation
            return ((pid1+ff1));
        }
        public double ARM_Control_PID2() {
            int arm2Pos = arm2.getCurrentPosition();
            double pid2 = controller2.calculate(arm2Pos, (int) (target2 * ticks_in_degree_2)); //PID calculation
            double ff2 = 0; //feedforward calculation, change when equation is derived
            return ((pid2/* + ff2*/));
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            packet.addLine("time.seconds():"+(time.seconds()));
            packet.addLine("arm2pos:"+(arm2.getCurrentPosition()));
            packet.addLine("target2pos:"+target2);
            packet.addLine("target2pos:"+(int)(target2*ticks_in_degree_2));
            if (time.seconds() < runTime+waitTime) {
                packet.addLine("still running 2");
                if (time.seconds()>waitTime){
                    double power1 = ARM_Control_PID1();
                    double power2 = ARM_Control_PID2();
                    packet.addLine("power1:"+power1);
                    packet.addLine("power2:"+power2);
                    arm1.setPower(power1);
                    arm2.setPower(power2);}
                else{
                    arm1.setPower(0);
                    arm2.setPower(0);
                }
                return true;
            } else {
                arm1.setPower(0);
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftHighBasket() {return new waitLiftTarget(97.854286777,180.492048747);}
    public Action liftRung() {return new waitLiftTarget(3.4193,95.3431);}
    public Action liftWall() {return new waitLiftTarget(12.0513,155.7743);}
    public Action liftLowBasket() {return new waitLiftTarget(50,50);} //not tested i think
    public Action liftFloor() {return new waitLiftTarget(2.6303,163.6641);}
    public Action liftDown() {return new waitLiftTarget(4.48338159887,5.0199819357);}
    public Action waitLiftHighBasket() {return new waitLiftTarget(97.854286777,180.492048747,1);}
    public Action waitLiftRung() {return new waitLiftTarget(3.4193,95.3431,1);}
    public Action waitLiftWall() {return new waitLiftTarget(12.0513,155.7743,1);}
    public Action waitLiftLowBasket() {return new waitLiftTarget(50,50,1);} //not tested i think
    public Action waitLiftFloor() {return new waitLiftTarget(2.6303,163.6641,1);}
    public Action waitLiftDown() {return new waitLiftTarget(4.48338159887,5.0199819357,1);}
    public Action waitLiftHighBasket(double waitseconds) {return new waitLiftTarget(97.854286777,180.492048747,waitseconds);}
    public Action waitLiftRung(double waitseconds) {return new waitLiftTarget(3.4193,95.3431,waitseconds);}
    public Action waitLiftWall(double waitseconds) {return new waitLiftTarget(12.0513,155.7743,waitseconds);}
    public Action waitLiftLowBasket(double waitseconds) {return new waitLiftTarget(50,50,waitseconds);} //not tested i think
    public Action waitLiftFloor(double waitseconds) {return new waitLiftTarget(2.6303,163.6641,waitseconds);}
    public Action waitLiftDown(double waitseconds) {return new waitLiftTarget(4.48338159887,5.0199819357,waitseconds);}
    public Action waitLiftHighBasket(double waitseconds, double seconds) {return new waitLiftTarget(97.854286777,180.492048747,waitseconds,seconds);}
    public Action waitLiftRung(double waitseconds, double seconds) {return new waitLiftTarget(3.4193,95.3431,waitseconds,seconds);}
    public Action waitLiftWall(double waitseconds, double seconds) {return new waitLiftTarget(12.0513,155.7743,waitseconds,seconds);}
    public Action waitLiftLowBasket(double waitseconds, double seconds) {return new waitLiftTarget(50,50,waitseconds,seconds);} //not tested i think
    public Action waitLiftFloor(double waitseconds, double seconds) {return new waitLiftTarget(2.6303,163.6641,waitseconds,seconds);}
    public Action waitLiftDown(double waitseconds, double seconds) {return new waitLiftTarget(4.48338159887,5.0199819357,waitseconds,seconds);}
}