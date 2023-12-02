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
    public DrawbridgeState drawbridgeState = DrawbridgeState.ONE;
    public CollectorState collectorState = CollectorState.OFF;

    private static final double Level1 = 2088;
    private static final double Level2 = 1931;
    private static final double Level3 = 1802;
    private static final double Level4 = 1613;
    private static final double Level5 = 1433;
    public CollectorAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        CollectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Collector"));
        DrawbridgeServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "Drawbridge"));
        DrawbridgeServo.setPwmRange(new PwmControl.PwmRange(Level5, Level1));

    }


    public enum DrawbridgeState {
        ONE, TWO, THREE, FOUR, FIVE
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

                return true;
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
            case ONE: {
                DrawbridgeOne();
                break;
            }
            case TWO: {
                setDrawbridgeTwo();
                break;
            }
            case THREE: {
                setDrawbridgeThree();
                break;
            }
            case FOUR: {
                setDrawbridgeFour();
                break;
            }
            case FIVE: {
                setDrawbridgeFive();
                break;
            }

        }
    }

        public void setDrawbridgeOne(){
        drawbridgeState = DrawbridgeState.ONE;
        }
         public void setDrawbridgeTwo(){
        drawbridgeState = DrawbridgeState.TWO;
    }
        public void setDrawbridgeThree(){
        drawbridgeState = DrawbridgeState.THREE;
    }
        public void setDrawbridgeFour(){
        drawbridgeState = DrawbridgeState.FOUR;
    }
        public void setDrawbridgeFive(){
        drawbridgeState = DrawbridgeState.FIVE;
    }

    public void DrawbridgeOne(){
        DrawbridgeServo.setPosition(1);
    }
    public void DrawbridgeTwo(){
        DrawbridgeServo.setPosition(Level2/(Level1-Level5));
    }
    public void DrawbridgeThree(){
        DrawbridgeServo.setPosition(Level3/(Level1-Level5));
    }
    public void DrawbridgeFour(){
        DrawbridgeServo.setPosition(Level4/Level1-Level5);
    }
    public void DrawbridgeFive(){
        DrawbridgeServo.setPosition(0);
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
        CollectorMotor.setPower(1);
    }
    private void collectorOut() { CollectorMotor.setPower(-1);
    }

}
