package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TunePID;

public class ARM2_NEW {
    private DcMotor arm2;
    //PID controllers for ARM1 and ARM2
    private PIDController controller2;
    //PIDF gains
    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
    double f2 = TunePID.f2;
    //ticks to degrees conversion, very useful
    private final double ticks_in_degree_2 = 145.1 * 28 / 360; // = 11.2855555556
    private final double L1 = 43.2;
    private final double L2 = 43.2;
    private final double x1 = 36.96;
    private final double x2 = 26.4;
    private final double m1 = 810;
    private final double m2 = 99.79;

//    public static double getTarget2() {
//        return target2;
//    }

    public ARM2_NEW(HardwareMap hardwareMap) {
        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller2 = new PIDController(p2, i2, d2);
    }
    public class LiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target2;
        public LiftTarget(double pos) {
            target2 = pos;
            start = false;
        }
        public double ARM_Control_PID() {
//            double target1 = ARM1_NEW.getTarget1();
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
            if (/*Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 30 && */time.seconds() < 2) {
                packet.addLine("still running 2");
                double power = ARM_Control_PID();
                packet.addLine("power2:"+power);
                arm2.setPower(power);
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftHighBasket() {return new LiftTarget(180.492048747);}
    public Action liftRung() {return new LiftTarget(95.3431);}
    public Action liftWall() {return new LiftTarget(155.7743);}
    public Action liftLowBasket() {return new LiftTarget(50);} //not tested i think
    public Action liftFloor() {return new LiftTarget(163.6641);}
    public Action liftDown() {return new LiftTarget(5.0199819357);}

    public class waitLiftTarget implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double target2;
        public waitLiftTarget(double pos) {
            target2 = pos;
            start = false;
        }
        public double ARM_Control_PID() {
//            double target1 = ARM1_NEW.getTarget1();
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
            if (/*Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 30 && */time.seconds() < 3) {
                packet.addLine("still running 2");
                if (time.seconds()>1){
                    double power = ARM_Control_PID();
                    packet.addLine("power2:"+power);
                    arm2.setPower(power);}
                else{
                    arm2.setPower(0);
                }
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action waitLiftHighBasket() {return new waitLiftTarget(180.492048747);}
    public Action waitLiftRung() {return new waitLiftTarget(95.3431);}
    public Action waitLiftWall() {return new waitLiftTarget(155.7743);}
    public Action waitLiftLowBasket() {return new waitLiftTarget(50);} //not tested i think
    public Action waitLiftFloor() {return new waitLiftTarget(163.6641);}
    public Action waitLiftDown() {return new waitLiftTarget(5.0199819357);}
    //old code, do not delete until new code tested
//    public class LiftRung implements Action {
//        ElapsedTime time2 = new ElapsedTime();
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            target2 = 95.3431;
//            ARM_Control_PID(target2);
//            packet.addLine("time.seconds():"+(time2.seconds()-time2.startTime()));
//            if (/*Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 30 && */(time2.seconds()-time2.startTime()/1000.) < 2) {
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action liftRung() {
//        return new LiftRung();
//    }
//    public class LiftLowBasket implements Action {
//        ElapsedTime time = new ElapsedTime();
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            target2 = 50;
//            ARM_Control_PID(target2);
//            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 30 && time.seconds() < 2) {
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action liftLowBasket() {return new LiftLowBasket();}
//
//    public class LiftWall implements Action {
//        ElapsedTime time = new ElapsedTime();
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            target2 = 155.7743;
//            ARM_Control_PID(target2);
//            packet.addLine("time.seconds():"+(time.seconds()-time.startTime()));
//            if (/*Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 30 || */(time.seconds()-time.startTime()/1000.) < 1.5/*&& time.seconds() < 5*/) {
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action liftWall() {
//        return new LiftWall();
//    }
////    public class HookSpecimen implements Action {
////        double target2 = 50;
////        @Override
////        public boolean run(@NonNull TelemetryPacket packet) {
////            ARM_Control_PID(target2);
////            if (Math.abs(arm2.getCurrentPosition()-target2) > 15) {
////                return true;
////            } else {
////                arm2.setPower(0);
////                return false;
////            }
////        }
////    }
////    public Action hookSpecimen() {
////        return new HookSpecimen();
////    }
//    public class LiftFloor implements Action {
//        ElapsedTime time = new ElapsedTime();
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            target2 = 163.6641;
//            ARM_Control_PID(target2);
//            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 30 && time.seconds() < 2) {
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action liftFloor(){
//        return new LiftFloor();
//    }
//
//    public class LiftDown implements Action {
//        ElapsedTime time = new ElapsedTime();
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            target2 = 5.0199819357;
//            ARM_Control_PID(target2);
//            packet.addLine("time.seconds():"+(time.seconds()-time.startTime()));
//            if (/*Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 30 */ (time.seconds()-time.startTime()/1000.) < 2/*&& time.seconds() < 5*/) {
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action liftDown(){
//        return new LiftDown();
//    }
//    public class LiftHighBasket implements Action {
//        ElapsedTime time = new ElapsedTime();
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            target2 = 180.492048747;
//            ARM_Control_PID(target2);
//            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 30 && time.seconds() < 2) {
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action liftHighBasket() {return new LiftHighBasket();}
//}
}