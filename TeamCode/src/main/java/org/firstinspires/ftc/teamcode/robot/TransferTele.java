package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

public class TransferTele {
    private Telemetry telemetry;
    private final DcMotorEx transferMotor;
    private HardwareMap hardwareMap;
    public TransferTele(HardwareMap hardwareMap,Telemetry telemetry) {
        this.telemetry=telemetry;
        this.hardwareMap=hardwareMap;

        transferMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class,"Transfer"));
    }
    public enum TransferState {
        OFF, IN, OUT
    }



    TransferState transferState = TransferState.OFF;
    
    public void transferState(){
        switch (transferState) {
            case OFF: {
                transferOff();
                break;
            }
            case IN: {
                transferIn();
                break;
            }
            case OUT: {
                transferOut();
                break;
            }
        }
    }

    public void setTransferOff(){
        transferState = TransferState.OFF;
    }

    public void setTransferIn(){
        transferState = TransferState.IN;
    }

    public void setTransferOut(){
        transferState = TransferState.OUT;
    }
    private void transferOff(){
        transferMotor.setPower(0);
    }
    private void transferIn(){
        transferMotor.setPower(0.5);
    }
    private void transferOut(){
        transferMotor.setPower(-0.5);
    }
}
