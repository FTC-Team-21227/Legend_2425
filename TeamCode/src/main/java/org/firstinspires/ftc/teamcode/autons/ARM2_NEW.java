package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TunePID;

public class ARM2_NEW {
    private DcMotor arm2;
    //PID controllers for ARM1 and ARM2
    private PIDController controller2;
    //PIDF gains
    double p2 = TunePID.p2, i2 = TunePID.i2, d2 = TunePID.d2;
    double f2 = TunePID.f2;
    //ticks to degrees conversion, very useful
    private final double ticks_in_degree_2 = 145.1*28/360; // = 11.2855555556

    public ARM2_NEW(HardwareMap hardwareMap) {
        arm2 = hardwareMap.get(DcMotor.class, "ARM2");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller2 = new PIDController(p2, i2, d2);
    }
    public void ARM_Control_PID(double target2){
        int arm2Pos = arm2.getCurrentPosition();
        double pid2 = controller2.calculate(arm2Pos,(int)(target2*ticks_in_degree_2)); //PID calculation
        double ff2 = Math.cos(Math.toRadians(target2)) * f2; // feedforward calculation, change when equation is derived
        double power2 = (pid2 + ff2)/1.5;
        arm2.setPower(power2); //set the power
    }
    //action names and values need to be updated.
    public class LiftRung implements Action {
        double target2 = 158.644092101;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target2);
            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 15) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftRung() {
        return new LiftRung();
    }
    public class LiftLowBasket implements Action {
        double target2 = 50;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target2);
            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 15) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftLowBasket() {return new LiftLowBasket();}

    public class LiftWall implements Action {
        double target2 = 155.7743;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target2);
            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 15) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftWall() {
        return new LiftWall();
    }
//    public class HookSpecimen implements Action {
//        double target2 = 50;
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            ARM_Control_PID(target2);
//            if (Math.abs(arm2.getCurrentPosition()-target2) > 15) {
//                return true;
//            } else {
//                arm2.setPower(0);
//                return false;
//            }
//        }
//    }
//    public Action hookSpecimen() {
//        return new HookSpecimen();
//    }
    public class LiftFloor implements Action {
        double target2 = 163.6641;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target2);
            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 15) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftFloor(){
        return new LiftFloor();
    }

    public class LiftDown implements Action {
        double target2 = 5.0199819357;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target2);
            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 15) {
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
    public class LiftHighBasket implements Action {
        double target2 = 180.492048747;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target2);
            if (Math.abs(arm2.getCurrentPosition()-(int)(target2*ticks_in_degree_2)) > 15) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftHighBasket() {return new LiftHighBasket();}
}