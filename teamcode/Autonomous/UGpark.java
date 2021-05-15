package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.GorillabotsCentral;


@Autonomous(group = "test", name = "UGpark")

public class UGpark extends GorillabotsCentral {
    public void runOpMode() {

        initializeComponentsAutonomous();

        waitForStart();

        MoveUntilTime(1000,180,0.6);//180 DEGREES IS FORWARDS, 0 IS BACK
        stopMotors();
        sleep(10000);
        MoveUntilTime(1000,180,1);
        stopMotors();
        sleep(200);





        if(!opModeIsActive())
        {
            return;
        }
        sleep(28000);//time until end of auto period

        return;
    }

}
