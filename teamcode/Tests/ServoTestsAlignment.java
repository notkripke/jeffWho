package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GorillabotsCentral;


@TeleOp(group="tests", name="ServoTestAlignment")
public class ServoTestsAlignment extends GorillabotsCentral
{

    @Override
    public void runOpMode() {
        double position = 0.5;

        boolean increaseWatch = false;
        boolean decreaseWatch = false;

        initializeComponents();

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.right_bumper && !increaseWatch) {
                position = position + .01;
            }
            increaseWatch = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !decreaseWatch) {
                position = position - .01;
            }
            decreaseWatch = gamepad1.left_bumper;

            if (gamepad1.right_trigger > .5) {
                position = position + .1;
            }
            if (gamepad1.left_trigger > .5) {
                position = position - .1;
            }

            if (position > 1) {
                position = 1;
            }
            if (position < 0) {
                position = 0;
            }

            robot.Guard(position);

            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}