package org.firstinspires.ftc.teamcode.Components;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware
{
    Telemetry tele;

    public DcMotor ShooterMotor, Intake, WobbleArm, Transfer;
    public Servo Guard;

    public static final double guardUp = 0;
    public static final double guardDown = 0.68;

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry)
    {
        tele = telemetry;

        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        Intake = hardwareMap.dcMotor.get("Intake");
        WobbleArm = hardwareMap.dcMotor.get("WobbleArm");
        Transfer = hardwareMap.dcMotor.get("Transfer");
        Guard = hardwareMap.servo.get("Guard");

        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Guard(double position){
        Guard.setPosition(position);
    }
    

}
