package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotsCentral;
@TeleOp(group = "test", name = "GyroTest")
public class GyroTest extends GorillabotsCentral {
    public void runOpMode() {
        initializeComponents();
        double x = 0;
        double r = 0;
        double y = 0;
        waitForStart();
        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;
            drive.go(.8*x,.8*y,.8*r);
            if(gamepad1.a){
                gyro.resetAngle();
            }
            telemetry.addData("Gyro:", gyro.getAngle());
            telemetry.update();
        }
    }
}