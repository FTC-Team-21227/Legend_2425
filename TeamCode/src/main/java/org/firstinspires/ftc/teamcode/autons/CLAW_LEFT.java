package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CLAW_LEFT {
    private Servo Claw;
    public CLAW_LEFT(HardwareMap hardwareMap) {
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.scaleRange(0.2,0.9);
    }

    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(1);
            return false;
        }
    }
    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(0);
            return false;
        }
    }
    public Action openClaw() {
        return new OpenClaw();
    }
    public class OpenSlightly implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw.setPosition(0.8);
            return false;
        }
    }
    public Action openSlightly() {
        return new OpenSlightly();
    }
}
