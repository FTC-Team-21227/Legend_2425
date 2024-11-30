package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp2425_PIDFArm;
import org.firstinspires.ftc.teamcode.TunePID;

public class ARM1_NEW {
    private DcMotor arm1;
    //PID controllers for ARM1 and ARM2
    private PIDController controller1;
    //PIDF gains
    double p1 = TunePID.p1, i1 = TunePID.i1, d1 = TunePID.d1;
    double f1 = TunePID.f1;
    //ticks to degrees conversion, very useful
    private final double ticks_in_degree_1 = 537.7*28/360; // = 41.8211111111

    public ARM1_NEW(HardwareMap hardwareMap) {
        arm1 = hardwareMap.get(DcMotor.class, "ARM1");
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller1 = new PIDController(p1, i1, d1);
    }
    public void ARM_Control_PID(double target1){
        int arm1Pos = arm1.getCurrentPosition();
        double pid1 = controller1.calculate(arm1Pos,(int)(target1*ticks_in_degree_1)); //PID calculation
        double ff1 = Math.cos(Math.toRadians(target1)) * f1; // feedforward calculation, change when equation is derived
        double power1 = (pid1 + ff1)/1.5;
        arm1.setPower(power1); //set the power
    }
    //action names and values need to be updated.
    public class LiftHighBasket implements Action {
        double target1 = 97.854286777;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target1);
            if (Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 15) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftHighBasket() {return new LiftHighBasket();}

    public class LiftHookSpecimen implements Action {
        double target1 = 50;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target1);
            if (Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 15) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftHookSpecimen() {
        return new LiftHookSpecimen();
    }
    public class LiftRung implements Action {
        double target1 = 36.8354632163;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target1);
            if (Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 15) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftRung() {
        return new LiftRung();
    }
    public class LiftWall implements Action {
        double target1 = 12.0513;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target1);
            if (Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 15) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftWall() {
        return new LiftWall();
    }


    public class LiftLowBasket implements Action {
        double target1 = 50;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target1);
            if (Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 15) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftLowBasket() {return new LiftLowBasket();}
    public class LiftFloor implements Action {
        double target1 = 2.6303;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target1);
            if (Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 15) {
                return true;
            } else {
                arm1.setPower(0);
                return false;
            }
        }
    }
    public Action liftFloor() {return new LiftFloor();}
    public class LiftDown implements Action {
        double target1 = 4.48338159887;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            ARM_Control_PID(target1);
            if (Math.abs(arm1.getCurrentPosition()-(int)(target1*ticks_in_degree_1)) > 15) {
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