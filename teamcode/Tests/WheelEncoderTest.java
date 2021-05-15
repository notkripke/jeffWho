package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotsCentral;
@TeleOp(group = "test", name = "WheelEncoderTest")
public class WheelEncoderTest extends GorillabotsCentral {
    public void runOpMode() {
        initializeComponents();
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                drive.resetDrivenDistance();
            }
        telemetry.addData("BL",drive.mbl.getCurrentPosition());
        telemetry.addData("BR",drive.mbr.getCurrentPosition());
        telemetry.addData("FR",drive.mfr.getCurrentPosition());
        telemetry.addData("FL",drive.mfl.getCurrentPosition());
        telemetry.update();
    }}}