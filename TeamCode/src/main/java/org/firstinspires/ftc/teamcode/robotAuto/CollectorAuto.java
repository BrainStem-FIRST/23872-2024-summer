package org.firstinspires.ftc.teamcode.robotAuto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;

public class CollectorAuto {
    private HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final DcMotorEx CollectorMotor;
    private final ServoImplEx DrawbridgeServo;
    public DrawbridgeState drawbridgeState = DrawbridgeState.DOWN;
    public CollectorState collectorState = CollectorState.OFF;
    private static final double Down = 1665;
    private static final double Up = 740;
    public CollectorAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        CollectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Collector"));
        DrawbridgeServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "Drawbridge"));
        DrawbridgeServo.setPwmRange(new PwmControl.PwmRange(Down, Up));

    }


    public enum DrawbridgeState {
        DOWN, UP
    }

    public enum CollectorState {
        OFF, IN, OUT
    }

    public Action collectorOutAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    collectorOut();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action collectorInAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    collectorIn();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action collectorOffAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    collectorOff();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action drawbridgeUpAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setDrawbridgeUp();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public void setCollectorState() {
        switch (collectorState) {
            case OFF: {
                collectorOff();
                break;
            }
            case IN: {
                collectorIn();
                break;
            }
            case OUT: {
                collectorOut();
                break;
            }
        }
    }

    public void setDrawbridgeState() {
        telemetry.addData("DrawbridgeState", drawbridgeState);
        switch (drawbridgeState){
            case DOWN: {
                setDrawbridgeDown();
                break;
            }
            case UP: {
                setDrawbridgeUp();
                break;
            }
        }
    }

        public void setDrawbridgeUp(){
        drawbridgeState = DrawbridgeState.UP;
        }
         public void setDrawbridgeDown(){
        drawbridgeState = DrawbridgeState.DOWN;
    }
        public void setCollectorOff(){
        collectorState = CollectorState.OFF;
    }

    public void setCollectorIn(){
        collectorState = CollectorState.IN;
    }

    public void setCollectorOut(){
        collectorState = CollectorState.OUT;
    }

    private void collectorOff() {CollectorMotor.setPower(0);}
    private void collectorIn(){
        CollectorMotor.setPower(0.1);
    }
    private void collectorOut() { CollectorMotor.setPower(-0.1);
    }

}
