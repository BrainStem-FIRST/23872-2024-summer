package org.firstinspires.ftc.teamcode.robotTele;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.CachingServo;

public class CollectorTele {
    private HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final DcMotorEx CollectorMotor;
//    private final ServoImplEx DrawbridgeServo;
    private final ServoImplEx MonkServo;
    public MonkState monkState = MonkState.RAISED;

    private static final double MONK_RAISED =1125 ;
    private static final double MONK_DOWN =2316 ;
//    public DrawbridgeState drawbridgeState = DrawbridgeState.ONE;
    public CollectorState collectorState = CollectorState.OFF;

//    private static final double Level1 = 1800;
//    private static final double Level2 = 1600;
//    private static final double Level3 = 1300;
//    private static final double Level4 = 1000;
//    private static final double Level5 = 650;
    public CollectorTele(HardwareMap hardwareMap,Telemetry telemetry) {
        this.telemetry = telemetry;
        CollectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Collector"));
        MonkServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "monkServo"));
        MonkServo.setPwmRange(new PwmControl.PwmRange(MONK_RAISED,MONK_DOWN ));
    }

    public enum MonkState {
        RAISED, DOWN
    }

    public enum CollectorState {
        OFF, IN, OUT
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

    public void setMonkState() {
        telemetry.addData("Monk", monkState);
        telemetry.update();
        switch (monkState){
            case RAISED: {
                MonkRaised();
                break;
            }
//            case TWO: {
//                DrawbridgeTwo();
//                break;
//            }
//            case THREE: {
//                DrawbridgeThree();
//                break;
//            }
//            case FOUR: {
//                DrawbridgeFour();
//                break;
//            }
            case DOWN:
                MonkDown();
                break;
            }

        }


        public void setMonkRaised() {monkState = MonkState.RAISED;}
        public void setMonkDown(){
        monkState = MonkState.DOWN;
    }

    public void MonkRaised(){MonkServo.setPosition(0.99);}
    public void MonkDown(){MonkServo.setPosition(0.01);}


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
