package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.GorillabotsCentral;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Tracks
{
    Telemetry tele;
    MecanumDrive drive;
    RevGyro gyro;

    public static final double COUNTS_PER_MOTOR_REV = 160;     //12.5:1
    public static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public Tracks(HardwareMap hardwareMap, Telemetry telemetry) {
        tele = telemetry;
        drive = new MecanumDrive(hardwareMap, telemetry);
        gyro = new RevGyro(hardwareMap, telemetry);
    }
}
