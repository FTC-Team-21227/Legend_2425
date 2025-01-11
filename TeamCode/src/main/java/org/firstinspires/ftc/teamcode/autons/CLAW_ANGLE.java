package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CLAW_ANGLE {
    private Servo Claw_Angle;
    public CLAW_ANGLE(HardwareMap hardwareMap) {
        Claw_Angle = hardwareMap.get(Servo.class, "Claw_Angle");
        Claw_Angle.scaleRange(0.04, 0.7);
    }

    public class Forward implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        public Forward(){
            start = false;
            runTime = 0.5;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            } else {
                Claw_Angle.setPosition(0);
                return false;
            }
        }
    }
    public Action forward() {
        return new Forward();
    }

    public class Backward implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        public Backward(){
            start = false;
            runTime = 0.5;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            } else {
                Claw_Angle.setPosition(1);
                return false;
            }
        }
    }
    public Action backward() {
        return new Backward();
    }
    public class Backward2 implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        public Backward2(){
            start = false;
            runTime = 1;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            } else {
                Claw_Angle.setPosition(1);
                return false;
            }
        }
    }
    public Action backward2() {
        return new Backward2();
    }
}
