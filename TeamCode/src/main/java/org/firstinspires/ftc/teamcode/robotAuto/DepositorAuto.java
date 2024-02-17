package org.firstinspires.ftc.teamcode.robotAuto;

import static org.firstinspires.ftc.teamcode.robotTele.DepositorTele.DepositorServoState.RESTING;
import static org.firstinspires.ftc.teamcode.robotTele.DepositorTele.DepositorServoState.SCORING;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingServo;

public class DepositorAuto {
    private final Telemetry telemetry;
    private HardwareMap hardwareMap;
    public DepositorServoState depositorServoState = DepositorServoState.RESTING;
    public PixelState pixelState = PixelState.HOLD;
    private final ServoImplEx LeftDepositor;
    private final ServoImplEx RightDepositor;
    private final ServoImplEx TopPixHold;
    private final ServoImplEx BottomPixHold;
    private static final double LEFT_DEPOSITOR_MAX = 1950;
    private static final double LEFT_DEPOSITOR_MIN = 800;
    private static final double RIGHT_DEPOSITOR_MAX = 800;
    private static final double RIGHT_DEPOSITOR_MIN = 1950;
    private static final double TOP_PIX_HOLD_MAX = 1350;
    private static final double TOP_PIX_HOLD_MIN = 100;
    private static final double BOTTOM_PIX_HOLD_MAX = 1677;
    private static final double BOTTOM_PIX_HOLD_MIN = 100;

    public DepositorAuto(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry=telemetry;
        this.hardwareMap=hardwareMap;

        LeftDepositor = new CachingServo(hardwareMap.get(ServoImplEx.class,"LeftDepositor"));
        RightDepositor = new CachingServo(hardwareMap.get(ServoImplEx.class,"RightDepositor"));
        TopPixHold = new CachingServo(hardwareMap.get(ServoImplEx.class,"TopPixHold"));
        BottomPixHold = new CachingServo(hardwareMap.get(ServoImplEx.class,"BottomPixHold"));

        LeftDepositor.setPwmRange(new PwmControl.PwmRange(LEFT_DEPOSITOR_MAX, LEFT_DEPOSITOR_MIN));
        RightDepositor.setPwmRange(new PwmControl.PwmRange(RIGHT_DEPOSITOR_MAX, RIGHT_DEPOSITOR_MIN));
        TopPixHold.setPwmRange(new PwmControl.PwmRange(TOP_PIX_HOLD_MAX, TOP_PIX_HOLD_MIN));
        BottomPixHold.setPwmRange(new PwmControl.PwmRange(BOTTOM_PIX_HOLD_MAX, BOTTOM_PIX_HOLD_MIN));
    }


    public enum PixelState {
        HOLD, TOP_DROP, BOTTOM_DROP
    }

    public Action topPixelDropAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    topPixelDrop();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action bottomPixelDropAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    bottomPixelDrop();
                    initialized = true;
                }

                return false;
            }
        };
    }

   

    public Action pixelHoldAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pixelHold();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public void pixelState() {
        switch (pixelState){
            case HOLD: {
                pixelHold();
                break;
            }
            case TOP_DROP: {
                topPixelDrop();
                break;
            }
            case BOTTOM_DROP: {
                bottomPixelDrop();
                break;
            }
        }
    }

    public Action depositorScoringAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    depositorScoring();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action depositorRestingAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    depositorResting();
                    initialized = true;
                }

                return false;
            }
        };
    }
    public void pixelHold() {
        TopPixHold.setPosition(0.01);
        BottomPixHold.setPosition(0.01);
    }
    private void topPixelDrop() {
        TopPixHold.setPosition(0.7);
    }
    private void bottomPixelDrop() {
        BottomPixHold.setPosition(0.7);
    }
    public enum DepositorServoState {
        RESTING, SCORING
    }

    public void setHoldState() {
        pixelState = PixelState.HOLD;
    }
    public void setTopDropState() {
        pixelState = PixelState.TOP_DROP;
    }
    public void setBottomDropState() {
        pixelState = PixelState.BOTTOM_DROP;
    }

    public void depositorServoState(LiftAuto lift) {
        switch (depositorServoState){
            case RESTING: {
                depositorResting();
                break;
            }
            case SCORING:{
                depositorScoring();
                break;
            }
        }
    }

    private void setDepositorState(LiftAuto lift){
        if (lift.liftState == LiftAuto.LiftState.ZERO){
            depositorServoState = DepositorServoState.RESTING;
        } else {
            depositorServoState = DepositorServoState.SCORING;
        }
    }
    private void depositorResting(){
        LeftDepositor.setPosition(0.99);
        RightDepositor.setPosition(0.99);
        telemetry.addData("ServoState", "Resting");
    }
    private void depositorScoring(){
        LeftDepositor.setPosition(0.007);
        RightDepositor.setPosition(0.007);
        telemetry.addData("ServoState", "Scoring");
    }

    public void setRestingState() {
        depositorServoState = DepositorServoState.RESTING;
    }
    public void setScoringState() {
        depositorServoState = DepositorServoState.SCORING;
    }


}

