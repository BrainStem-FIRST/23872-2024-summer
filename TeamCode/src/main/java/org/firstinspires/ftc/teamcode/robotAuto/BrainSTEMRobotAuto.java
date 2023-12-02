package org.firstinspires.ftc.teamcode.robotAuto;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BrainSTEMRobotAuto {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public CollectorAuto collector;
    public TransferAuto transfer;
    public DepositorAuto depositor;
    public LiftAuto lift;
    public HangingAuto hanging;

    public BrainSTEMRobotAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;


        collector = new CollectorAuto(hardwareMap, telemetry);
        hanging = new HangingAuto(hardwareMap, telemetry);
        transfer = new TransferAuto(hardwareMap, telemetry);
        depositor = new DepositorAuto(hardwareMap, telemetry);
        lift = new LiftAuto(hardwareMap, telemetry);
    }

    public void update() {
        telemetry.addData("collectorState", collector.collectorState);
        collector.setCollectorState();
        hanging.hangingState();
        hanging.setServoState();
        transfer.transferState();
        depositor.depositorServoState(lift);
        depositor.pixelState();
        lift.liftState();
        lift.updateLevel();
    }

    }


