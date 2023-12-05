package org.firstinspires.ftc.teamcode.robotTele;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.CachingServo;

public class DroneTele {
    private final ServoImplEx droneServo;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public ServoState servoState = ServoState.CLASP;

    public DroneTele(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        droneServo = new CachingServo(hardwareMap.get(ServoImplEx.class, "DroneServo"));

        droneServo.setPwmRange(new PwmControl.PwmRange(DroneServo_MAX, DroneServo_MIN));

    }

    private static final double DroneServo_MAX = 1840;
    private static final double DroneServo_MIN = 1500;


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
