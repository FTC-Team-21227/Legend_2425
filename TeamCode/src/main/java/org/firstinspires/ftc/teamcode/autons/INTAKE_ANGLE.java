package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class INTAKE_ANGLE {
    private Servo Intake_Angle;

    public INTAKE_ANGLE(HardwareMap hardwareMap) {
        Intake_Angle = hardwareMap.get(Servo.class, "Intake_Angle");
    }

    public class RotatePosition0 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Intake_Angle.setPosition(0.65);
            return false;
        }
    }
    public Action RotatePosition0() {
        return new RotatePosition0();
    }

    public class RotatePosition1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Intake_Angle.setPosition(0.98);
            return false;
        }
    }
    public Action RotatePosition1() {
        return new RotatePosition1();
    }
}