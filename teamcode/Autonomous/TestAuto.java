package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.GorillabotsCentral;

@Autonomous(group = "test", name = "TestAuto")

public class TestAuto extends GorillabotsCentral {
    public void runOpMode() {

        DcMotor Intake;
        Intake = hardwareMap.dcMotor.get("Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor ShooterMotor;
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor Transfer;
        Transfer = hardwareMap.dcMotor.get("Transfer");
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        initializeComponentsAutonomous();
        waitForStart();

        gyro.resetAngle();
    MoveUntilEncoder(100,140,.5);
    sleep(500);
    rampShooter(.58);
    TurnAbsolute(-11,.3,.6);
    sleep(2000);
    moveTransfer();
    sleep(100);
    stopTransfer();
    TurnAbsolute(-16.7,.3,.6);
    sleep(1300);
    moveTransfer();
    sleep(100);
    stopTransfer();
    sleep(300);
    TurnAbsolute(-23,.3,.6);
    sleep(800);
    reverseIntake();
    sleep(200);
    moveTransfer();
    sleep(700);
    stopTransfer();
    stopShooter();

        if(!opModeIsActive())
        {
            return;
        }

        return;
    }

}
