package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.CachingServo;

public class Depositor {
    Telemetry telemetry;
     HardwareMap hardwareMap;

     public DepositorServoState depositorServoState = DepositorServoState.RESTING;
     private ServoImplEx LeftDepositor;
    private ServoImplEx RightDepositor;

    private static final double LEFT_DEPOSITOR_MAX = 1500;
    private static final double LEFT_DEPOSITOR_MIN = 500;
    private static final double RIGHT_DEPOSITOR_MAX = 1500;
    private static final double RIGHT_DEPOSITOR_MIN = 500;


    public Depositor(HardwareMap hardwareMap,Telemetry telemetry){
        this.telemetry=telemetry;
        this.hardwareMap=hardwareMap;

        LeftDepositor = new CachingServo(hardwareMap.get(ServoImplEx.class,"LeftDepositor"));
        RightDepositor = new CachingServo(hardwareMap.get(ServoImplEx.class,"RightDepositor"));

        LeftDepositor.setPwmRange(new PwmControl.PwmRange(LEFT_DEPOSITOR_MAX, LEFT_DEPOSITOR_MIN));
        RightDepositor.setPwmRange(new PwmControl.PwmRange(RIGHT_DEPOSITOR_MAX, RIGHT_DEPOSITOR_MIN));



    }

    public enum DepositorServoState {
        RESTING, SCORING
    }
    public void depositorServoState() {
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


    private void depositorResting(){
        LeftDepositor.setPosition(0.01);
        RightDepositor.setPosition(0.01);
    }
    private void depositorScoring(){
        LeftDepositor.setPosition(0.99);
        RightDepositor.setPosition(0.99);
    }

    public void setRestingState() {depositorServoState = DepositorServoState.RESTING;}
    public void setScoringState() {depositorServoState = DepositorServoState.SCORING;}


}

