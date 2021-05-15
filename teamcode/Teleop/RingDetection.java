package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.VisionPipeline;
import org.firstinspires.ftc.teamcode.GorillabotsCentral;

@TeleOp(group = "AAAAAAAAAA", name = "RingDetection")

public class RingDetection extends GorillabotsCentral {
    public void runOpMode() {
        initializeComponents();
        double x = 0;
        double r = 0;
        double y = 0;
        startVisionProcessing();
        webcam.openCameraDevice();


        waitForStart();

       while (opModeIsActive()) {
           VisionPipeline.RingPosition numRings = pipeline.numRings;
           x = gamepad1.left_stick_x;
           y = -gamepad1.left_stick_y;
           r = gamepad1.right_stick_x;

           drive.go(-x,-y,-r);
            if (numRings == VisionPipeline.RingPosition.FOUR) {
                telemetry.addData("Rings:", pipeline.getPos());
            } else if (numRings == VisionPipeline.RingPosition.ONE) {
                telemetry.addData("Rings:", pipeline.getPos());
            } else {
                telemetry.addData("Rings:", pipeline.getPos());

            }

            telemetry.addData("avg1:",pipeline.getAnalysis());
            telemetry.update();


        }

    }
}
