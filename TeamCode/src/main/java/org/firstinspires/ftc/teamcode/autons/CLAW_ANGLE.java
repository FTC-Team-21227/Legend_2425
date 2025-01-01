package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CLAW_ANGLE {
    private Servo Claw_Angle;
    public CLAW_ANGLE(HardwareMap hardwareMap) {
        Claw_Angle = hardwareMap.get(Servo.class, "Claw_Angle");
        Claw_Angle.scaleRange(0.04, 0.7);
    }

    public class Forward implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw_Angle.setPosition(0);
            return false;
        }
    }
    public Action forward() {
        return new Forward();
    }

    public class Backward implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Claw_Angle.setPosition(1);
            return false;
        }
    }
    public Action backward() {
        return new Backward();
    }
}
