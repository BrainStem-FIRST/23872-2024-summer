package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.CachingServo;

public class Drone {
    private final ServoImplEx droneServo;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public ServoState servoState = ServoState.CLASP;

    public Drone(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        droneServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "DroneServo"));
    }

    public enum ServoState {
        CLASP, RELEASE
    }

    public void setServoState() {
        switch (servoState) {
            case CLASP: {
                claspServo();
                break;
            }
            case RELEASE: {
                releaseServo();
                break;
            }
        }
    }
    public void setClaspServo(){
        servoState = ServoState.CLASP;
    }

    public void setReleaseServo(){
        servoState = ServoState.RELEASE;
    }
    public void claspServo() {
        droneServo.setPosition(0.01);
    }

    public void releaseServo() {
        droneServo.setPosition(0.99);
    }

}
