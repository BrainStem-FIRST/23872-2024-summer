package org.firstinspires.ftc.teamcode.robotAuto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingMotor;

public class TransferAuto {
    private Telemetry telemetry;
    private final DcMotorEx transferMotor;
    private HardwareMap hardwareMap;

    public TransferAuto.TransferState transferState = TransferAuto.TransferState.OFF;
    public TransferAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry=telemetry;
        this.hardwareMap=hardwareMap;

        transferMotor = new CachingMotor(hardwareMap.get(DcMotorEx.class,"Transfer"));
    }
    public enum TransferState {
        OFF, IN, OUT
    }
    public Action transferInAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    transferIn();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action transferOffAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    transferOff();
                    initialized = true;
                }

                return false;
            }
        };
    }

    public Action transferOutAction() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    transferOut();
                    initialized = true;
                }

                return false;
            }
        };
    }


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
        transferMotor.setPower(1);
    }
    private void transferOut(){
        transferMotor.setPower(-1);
    }
}
