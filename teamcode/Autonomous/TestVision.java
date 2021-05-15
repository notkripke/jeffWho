package org.firstinspires.ftc.teamcode.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Components.VisionPipeline;
import org.firstinspires.ftc.teamcode.GorillabotsCentral;

@Autonomous(group = "test", name = "TestVision")

public class TestVision extends GorillabotsCentral {
    public void runOpMode() {
        int startloop = 0;
        initializeComponentsAutonomous();
        startVisionProcessing();
        webcam.openCameraDevice();
webcam.setPipeline(pipeline);
        int rings = 0;



while(startloop == 0){//startloop == 0
            telemetry.addData("Rings",  pipeline.getPos());
            telemetry.update();
            if(pipeline.getPos() == 0){
                rings = 0;
            }
            else if (pipeline.getPos() == 1){
                rings = 1;
            }
            else if (pipeline.getPos() == 4){
                rings = 4;
            }}
startloop = 1;

        waitForStart();

        switch (rings) {
            case 0:
                //Path for no rings
                telemetry.addData("Track", "0");
                telemetry.update();
                sleep(4000);
                break;
            case 1:
                //path for one ring
                telemetry.addData("Track", "1");
                telemetry.update();
                sleep(4000);
                break;
            case 4:
                //path for full stack
                telemetry.addData("Track", "4");
                telemetry.update();
                sleep(4000);
                break;



        }

        return;
    }

}
