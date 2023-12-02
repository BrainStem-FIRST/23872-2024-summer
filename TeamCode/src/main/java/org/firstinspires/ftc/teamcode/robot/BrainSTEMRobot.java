package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BrainSTEMRobot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public CollectorTele collector;
    public TransferTele transfer;
    public Depositor depositor;
    public Lift lift;
    public Hanging hanging;

    public BrainSTEMRobot (HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;


        collector = new CollectorTele(hardwareMap, telemetry);
        hanging = new Hanging(hardwareMap, telemetry);
        transfer = new TransferTele(hardwareMap, telemetry);
        depositor = new Depositor(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
    }

    public void update(){
        telemetry.addData("collectorState", collector.collectorState);
        collector.setCollectorState();
        hanging.hangingState();
        hanging.setServoState();
        transfer.transferState();
        depositor.depositorServoState();
//        lift.liftState();
    }



}

