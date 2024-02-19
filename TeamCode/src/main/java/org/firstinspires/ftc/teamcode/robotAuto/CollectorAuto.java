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

    private static final double A_UP = 100;
    private static final double A_4 = 1000;
    private static final double A_5 = 650;



    public CollectorAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        CollectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Collector"));
        DrawbridgeServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "Drawbridge"));
        DrawbridgeServo.setPwmRange(new PwmControl.PwmRange(A_4, A_UP));

    }


    public enum DrawbridgeState {
        A_UP, A_5, A_4
    }

    public enum CollectorState {
        OFF, IN, OUT, STACK_OUT, STACK_IN, STACK_OFF
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

    public Action collectorStackOffAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    collectorStackOff();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action collectorStackOutAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    collectorStackOut();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action collectorStackInAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    collectorStackIn();
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
            case STACK_IN: {
                collectorStackIn();
                break;
            }
            case STACK_OUT: {
                collectorStackOut();
                break;
            }
            case STACK_OFF: {
                collectorStackOff();
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

//    public Action Audience_4() {
//        return new Action() {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    setAutoDrawbridge_4();
//                    initialized = true;
//                }
//
//                return false;
//            }
//        };
//    }

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
                AutoDrawbridgeUp();
                break;
            }
//            case A_4: {
//                AutoDrawbridge_4();
//                break;
//            }
            case A_5: {
                AutoDrawbridge_5();
                break;
            }
        }
    }

    public void setAutoDrawbridgeUp() {drawbridgeState = DrawbridgeState.A_UP;}
//    public void setAutoDrawbridge_4() {
//        drawbridgeState = DrawbridgeState.A_4;
//    }
    public void setAutoDrawbridge_5() {
        drawbridgeState = DrawbridgeState.A_5;
    }

    public void AutoDrawbridgeUp() {DrawbridgeServo.setPosition(0.99);}
   // public void AutoDrawbridge_4() {DrawbridgeServo.setPosition(0.01);}
    public void AutoDrawbridge_5() {
        DrawbridgeServo.setPosition(0.01);
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

    public void setCollectorStackOff() {
        collectorState = CollectorState.OFF;
    }

    public void setCollectorStackIn() {
        collectorState = CollectorState.IN;
    }

    public void setCollectorStackOut() {
        collectorState = CollectorState.OUT;
    }

    private void collectorOff() {
        CollectorMotor.setPower(0);
    }

    private void collectorIn() {
        CollectorMotor.setPower(0.85);
    }

    private void collectorOut() {
        CollectorMotor.setPower(-0.25);
    }

    private void collectorStackOff() {
        CollectorMotor.setPower(0);
    }

    private void collectorStackIn() {
        CollectorMotor.setPower(1);
    }

    private void collectorStackOut() {
        CollectorMotor.setPower(-1);
    }

}