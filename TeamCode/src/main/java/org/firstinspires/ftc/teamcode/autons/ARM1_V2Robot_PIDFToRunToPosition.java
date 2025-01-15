package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TunePID;

public class ARM1_V2Robot_PIDFToRunToPosition {
    private DcMotor arm1;
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    //PIDF gains
    double p1 = TunePID.p1, i1 = TunePID.i1, d1 = TunePID.d1;
    double f1 = TunePID.f1;
    //ticks to degrees conversion
    private final double ticks_in_degree_1 = 41.8211111111;
    //length, COM, mass values for feedforward calculation
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;
    private final double highBasket = 97.854286777;
    private final double highRung = 3.3954;

    private final double wall = 15.0642;
    private final double wall2 = 80; //not tested
    private final double lowBasket = 50; //not tested
    private final double floor = 0.6217;
    private final double down = 4.48338159887;

    public ARM1_V2Robot_PIDFToRunToPosition(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(DcMotor.class, "ARM1");
        arm1.setDirection(DcMotor.Direction.REVERSE); //CHANGE BACK TO DCMOTORSIMPLE IF SOMETHING DOESN'T WORK
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller1 = new PIDController(p1, i1, d1);
    }
    public class LiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target1;
        double runTime;
        public LiftTarget(double pos){
            target1 = pos;
            start = false;
            runTime = 2;
        }
        public LiftTarget(double pos, double runtime){
            target1 = pos;
            start = false;
            runTime = runtime;
        }
        public double ARM_Control_PID(@NonNull TelemetryPacket packet /*NEW*/){
            double target2 = PoseStorage.target2; //NEW
            packet.addLine("target2pos1:"+target2); //NEW
            int arm1Pos = arm1.getCurrentPosition();
            double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
            double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
            /*NEW*/        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(target1))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
                            Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation
            return ((pid1+ff1));
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                PoseStorage.target1 = target1; //NEW
                time.reset();
                start = true;
            }
            packet.addLine("time.seconds():"+(time.seconds()));
            packet.addLine("arm1pos:"+(arm1.getCurrentPosition()));
            packet.addLine("target1pos:"+target1);
            packet.addLine("target1pos:"+(int)(target1*ticks_in_degree_1));
            if (time.seconds() < runTime) {
                packet.addLine("still running 1");
                double power = ARM_Control_PID(packet /*NEW*/);
                packet.addLine("power1:"+power);
                arm1.setPower(power);
                return true;
            } else {
                arm1.setTargetPosition((int)(target1*ticks_in_degree_1));
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
    }
    public Action liftHighBasket() {return new LiftTarget(highBasket);}
    public Action liftRung() {return new LiftTarget(highRung);}
    public Action liftWall() {return new LiftTarget(wall);}
    public Action liftWall2() {return new LiftTarget(wall2);}
    public Action liftLowBasket() {return new LiftTarget(lowBasket);} //not tested i think
    public Action liftFloor() {return new LiftTarget(floor);}
    public Action liftDown() {return new LiftTarget(down);}
    public Action liftHighBasket(double seconds) {return new LiftTarget(highBasket, seconds);}
    public Action liftRung(double seconds) {return new LiftTarget(highRung, seconds);}
    public Action liftWall(double seconds) {return new LiftTarget(wall, seconds);}
    public Action liftWall2(double seconds) {return new LiftTarget(wall2, seconds);}
    public Action liftLowBasket(double seconds) {return new LiftTarget(lowBasket, seconds);} //not tested i think
    public Action liftFloor(double seconds) {return new LiftTarget(floor, seconds);}
    public Action liftDown(double seconds) {return new LiftTarget(down, seconds);}


    public class waitLiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target1;
        double waitTime;
        double runTime;
        boolean startMove;
        public waitLiftTarget(double pos){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = 1;
            runTime = 2;
        }
        public waitLiftTarget(double pos, double tim){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = 2;
        }
        public waitLiftTarget(double pos, double tim, double runtime){
            target1 = pos;
            start = false;
            startMove = false;
            waitTime = tim;
            runTime = runtime;
        }
        public double ARM_Control_PID(@NonNull TelemetryPacket packet /*NEW*/){
            double target2 = PoseStorage.target2; //NEW
            packet.addLine("target2pos1:"+target2); //NEW
            int arm1Pos = arm1.getCurrentPosition();
            double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
            double ff1 = (m1*Math.cos(Math.toRadians(target1))*x1 +
            /*NEW*/        m2*Math.cos(Math.atan(((x2*Math.sin(Math.toRadians(target1+target2)))+(L1*Math.sin(Math.toRadians(Math.toRadians(target1)))))/((L1*Math.cos(Math.toRadians(target1)))+(x2*Math.cos(Math.toRadians(target1+target2))))))*
                            Math.sqrt(Math.pow((x2*Math.sin(Math.toRadians(target1+target2))+L1*Math.sin(Math.toRadians(target1))),2)+Math.pow((x2*Math.cos(Math.toRadians(target1+target2))+L1*Math.cos(Math.toRadians(target1))),2))) * f1; // feedforward calculation
            return ((pid1+ff1));
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            packet.addLine("time.seconds():"+(time.seconds()));
            packet.addLine("arm1pos:"+(arm1.getCurrentPosition()));
            packet.addLine("target1pos:"+target1);
            packet.addLine("target1pos:"+(int)(target1*ticks_in_degree_1));
            if (time.seconds() < runTime+waitTime) {
                packet.addLine("still running 1");
                if (time.seconds()>waitTime) {
                    //NEW
                    if (!startMove){
                        PoseStorage.target1 = target1;
                        startMove = true;
                    }
                    //
                    double power = ARM_Control_PID(packet /*new*/);
                    packet.addLine("power1:" + power);
                    arm1.setPower(power);
                }
                else{
                    arm1.setPower(0);
                }
                return true;
            } else {
                arm1.setTargetPosition((int) (target1*ticks_in_degree_1));
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return false;
            }
        }
    }
    public Action waitLiftHighBasket() {return new waitLiftTarget(highBasket);}
    public Action waitLiftRung() {return new waitLiftTarget(highRung);}
    public Action waitLiftWall() {return new waitLiftTarget(wall);}
    public Action waitLiftWall2() {return new waitLiftTarget(wall2);}
    public Action waitLiftLowBasket() {return new waitLiftTarget(lowBasket);} //not tested i think
    public Action waitLiftFloor() {return new waitLiftTarget(floor);}
    public Action waitLiftDown() {return new waitLiftTarget(down);}
    public Action waitLiftHighBasket(double waitseconds) {return new waitLiftTarget(highBasket, waitseconds);}
    public Action waitLiftRung(double waitseconds) {return new waitLiftTarget(highRung, waitseconds);}
    public Action waitLiftWall(double waitseconds) {return new waitLiftTarget(wall, waitseconds);}
    public Action waitLiftWall2(double waitseconds) {return new waitLiftTarget(wall2, waitseconds);}
    public Action waitLiftLowBasket(double waitseconds) {return new waitLiftTarget(lowBasket, waitseconds);} //not tested i think
    public Action waitLiftFloor(double waitseconds) {return new waitLiftTarget(floor, waitseconds);}
    public Action waitLiftDown(double waitseconds) {return new waitLiftTarget(down, waitseconds);}
    public Action waitLiftHighBasket(double waitseconds, double seconds) {return new waitLiftTarget(highBasket, waitseconds, seconds);}
    public Action waitLiftRung(double waitseconds, double seconds) {return new waitLiftTarget(highRung, waitseconds, seconds);}
    public Action waitLiftWall(double waitseconds, double seconds) {return new waitLiftTarget(wall, waitseconds, seconds);}
    public Action waitLiftWall2(double waitseconds, double seconds) {return new waitLiftTarget(wall2, waitseconds, seconds);}
    public Action waitLiftLowBasket(double waitseconds, double seconds) {return new waitLiftTarget(lowBasket, waitseconds, seconds);} //not tested i think
    public Action waitLiftFloor(double waitseconds, double seconds) {return new waitLiftTarget(floor, waitseconds, seconds);}
    public Action waitLiftDown(double waitseconds, double seconds) {return new waitLiftTarget(down, waitseconds, seconds);}
}