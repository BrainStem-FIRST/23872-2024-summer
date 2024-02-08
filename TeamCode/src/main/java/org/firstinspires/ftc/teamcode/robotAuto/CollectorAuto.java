package org.firstinspires.ftc.teamcode.robotAuto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotTele.LiftTele;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;

public class CollectorAuto {
    private HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final DcMotorEx CollectorMotor;
    private final ServoImplEx DrawbridgeServo;
    public DrawbridgeState drawbridgeState = DrawbridgeState.A_UP;
    public CollectorState collectorState = CollectorState.OFF;
    private static final double A_5 = 700;
    private static final double A_4 = 1000;
    private static final double A_UP = 100;

    public CollectorAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        CollectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Collector"));
        DrawbridgeServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "Drawbridge"));
        DrawbridgeServo.setPwmRange(new PwmControl.PwmRange(A_4, A_5, A_UP));

    }


    public enum DrawbridgeState {
        A_UP, A_5, A_4
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

    public Action Audience_Up() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setAutoDrawbridgeUp();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action Audience_4() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setAutoDrawbridge_4();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action Audience_5() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    setAutoDrawbridge_5();
                    initialized = true;
                }

                return false;
            }
        };
    }
    public void setDrawbridgeState() {
        telemetry.addData("DrawbridgeState", drawbridgeState);
        switch (drawbridgeState) {
            case A_UP: {
                setAutoDrawbridgeUp();
                break;
            }
            case A_4: {
                setAutoDrawbridge_4();
                break;
            }
            case A_5: {
                setAutoDrawbridge_5();
            }
        }
    }

    public void setAutoDrawbridgeUp() {drawbridgeState = DrawbridgeState.A_UP;
    }

    public void setAutoDrawbridge_4() {
        drawbridgeState = DrawbridgeState.A_4;
    }
    public void setAutoDrawbridge_5() {
        drawbridgeState = DrawbridgeState.A_5;
    }


    public void setCollectorOff() {
        collectorState = CollectorState.OFF;
    }

    public void setCollectorIn() {
        collectorState = CollectorState.IN;
    }

    public void setCollectorOut() {
        collectorState = CollectorState.OUT;
    }

    private void collectorOff() {
        CollectorMotor.setPower(0);
    }

    private void collectorIn() {
        CollectorMotor.setPower(0.4);
    }

    private void collectorOut() {
        CollectorMotor.setPower(-0.1);
    }

}