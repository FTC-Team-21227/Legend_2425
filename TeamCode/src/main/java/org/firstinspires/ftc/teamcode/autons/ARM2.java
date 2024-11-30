package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ARM2 {
    private DcMotorEx arm2;

    public ARM2(HardwareMap hardwareMap) {
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2"); //ARM2
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    //public

    public class LiftBucketUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 7508) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftBucketUp() {
        return new LiftBucketUp();
    }
    public class LiftRungUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 6293) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action LiftRungUp() {
        return new LiftRungUp();
    }
    public class LiftRungDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(-1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos >= 6293) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action LiftRungDown() {
        return new LiftRungDown();
    }


    public class LiftLowBasketUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(-1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos > 3340) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftLowBasketUp() {return new ARM2.LiftLowBasketUp();}




    public class LiftWallUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 6293) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftWallUp() {
        return new LiftWallUp();
    }
//    public class HookSpecimen implements Action {
//        private boolean initialized = false;
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                arm2.setPower(1);
//                initialized = true;
//            }
//
//            double pos = arm2.getCurrentPosition();
//            if (pos < 6732) {
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
    public class LiftFloorUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos < 6481) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action liftFloorUp(){
        return new LiftFloorUp();
    }
    public class LiftFloorDown implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(-1);
                initialized = true;
            }

            double pos = arm2.getCurrentPosition();
            if (pos > 6481) {
                return true;
            } else {
                arm2.setPower(0);
                return false;
            }
        }
    }
    public Action LiftFloorDown(){
        return new LiftFloorDown();
    }

    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                arm2.setPower(-1);
                initialized = true;
            }
            double pos = arm2.getCurrentPosition();
            if (pos > 800) {
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
}