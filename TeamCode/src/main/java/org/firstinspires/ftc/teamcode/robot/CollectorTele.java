package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.CollectorTele.CollectorState.OFF;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.CachingMotor;

public class CollectorTele {
    private final DcMotorEx collectorMotor;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public CollectorTele(HardwareMap hardwareMap,Telemetry telemetry){
        this.telemetry = telemetry;

        collectorMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class, "Collector"));
    }

    public enum CollectorState {
        OFF, IN, OUT
    }

    CollectorState collectorState = OFF;

    public void setCollectorState() {
        telemetry.addData("collectorState", collectorState);
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

    public void setCollectorOff(){
        collectorState = CollectorState.OFF;
    }

    public void setCollectorIn(){
        collectorState = CollectorState.IN;
    }

    public void setCollectorOut(){
        collectorState = CollectorState.OUT;
    }

    private void collectorOff() {collectorMotor.setPower(0);}
    private void collectorIn(){
        collectorMotor.setPower(0.5);
    }
    private void collectorOut() { collectorMotor.setPower(-0.5);
    }

}
