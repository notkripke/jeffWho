package org.firstinspires.ftc.teamcode.Components;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber {
    Telemetry tele;

    public Servo grabber;

    public static final double Grabber_Close = .24;
    public static final double Grabber_Open = .5;

    public Grabber(HardwareMap hardwareMap, Telemetry telemetry) {
        tele = telemetry;
        grabber = hardwareMap.get(Servo.class, "capstone");
    }
    public void grabber(double pos)
    {
        grabber.setPosition(pos);
    }
}