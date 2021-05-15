package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Grabber;
import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.GorillabotsCentral;


@TeleOp(group = "AAAAAAAAAA", name = "ShooterPowerTest")
public class ShooterPowerTest extends GorillabotsCentral {

    @Override
    public void runOpMode() {

        initializeComponents();

        double power = 80;
        double x = 0;
        double r = 0;
        double y = 0;

        waitForStart();
        boolean increaseWatch = false;
        boolean decreaseWatch = false;

        ElapsedTime SlowTimer = new ElapsedTime(); //creates timer to prevent rapid stage increase
        ElapsedTime IntakeTime = new ElapsedTime();
        ElapsedTime ShooterTime = new ElapsedTime();

        int slow = 0;
        int IntakeToggle = 0;
        int shootToggle = 0;

        //Servo grabber;
        //grabber = hardwareMap.get(Servo.class, "grabber");
        final double Grabber_Close = .6;
        final double Grabber_Open = .05;
        DcMotor ShooterMotor;
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor Transfer;
        Transfer = hardwareMap.dcMotor.get("Transfer");
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        DcMotor Intake;
        Intake = hardwareMap.dcMotor.get("Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor WobbleArm;
        WobbleArm = hardwareMap.dcMotor.get("WobbleArm");
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo grabber;
        grabber = hardwareMap.get(Servo.class, "grabber");


        while (opModeIsActive()) {

            // SET DRIVING STUFF ↓

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            if (gamepad2.y || gamepad1.y){;
                shootToggle = 0;
                telemetry.addData("Shooter","off");
            }
            if(gamepad1.dpad_down){
                ShooterMotor.setPower(-0.23);
                ShooterMotor.setPower(-0.23);
            }
            if(gamepad1.dpad_up) {
                ShooterMotor.setPower(0.79);
            }
            if(gamepad2.right_bumper){
                Intake.setPower(0.8);
            }
            if(gamepad1.right_trigger>0.3){
                robot.Guard(RobotHardware.guardDown);
            }
            if(gamepad1.left_trigger>0.3){
                robot.Guard(RobotHardware.guardUp);
            }
            if (gamepad1.right_bumper && !increaseWatch) {
                power = power + .01;
            }
            increaseWatch = gamepad1.right_bumper;

            if (gamepad1.left_bumper && !decreaseWatch) {
                power = power - .01;
            }
            decreaseWatch = gamepad1.left_bumper;

            if (gamepad1.right_trigger > .5) {
                power = power + .1;
            }
            if (gamepad1.left_trigger > .5) {
                power = power - .1;
            }

            if (power > 1) {
                power = 1;
            }
            if (power < 0) {
                power = 0;
            }

            if(gamepad2.right_trigger > 0.4 || gamepad2.left_trigger > 0.4){
                if(gamepad2.right_trigger > 0.4){
                    Transfer.setPower(.9);
                }
                if(gamepad2.left_trigger > 0.4){
                    Transfer.setPower(-0.6);
                }
            }
            if(!(gamepad2.right_trigger > 0.4 || gamepad2.left_trigger > 0.4)){
                Transfer.setPower(0);
            }
            if(gamepad2.dpad_down){
                WobbleArm.setPower(-0.75);
            }
            if(gamepad2.dpad_up) {
                WobbleArm.setPower(.65);
            }
            if(!gamepad2.dpad_up && !gamepad2.dpad_down){
                WobbleArm.setPower(0);
            }
            if(gamepad2.left_bumper && IntakeTime.time() > 1){
                IntakeToggle +=1;
                IntakeTime.reset();
            }

            if (gamepad1.b && SlowTimer.time() > 1.5) {
                slow += 1;
                SlowTimer.reset();
            }
            if(gamepad2.x && ShooterTime.time() > 1.5){
                shootToggle +=1;
                ShooterTime.reset();
            }


            switch(IntakeToggle){
                case 0:
                    Intake.setPower(0);
                    telemetry.addData("Intake","Off");
                    break;
                case 1:
                    Intake.setPower(-0.8);
                    telemetry.addData("Intake","On");
                    break;
                case 2:
                    IntakeToggle = 0;
                    break;
            }
            switch(shootToggle){
                case 0:
                    ShooterMotor.setPower(0);
                    telemetry.addData("shooter","Off");
                    break;
                case 1:
                    ShooterMotor.setPower(power); //.84
                    telemetry.addData("shooter","On");
                    break;
                case 2:
                    shootToggle = 0;
                    break;
            }
            telemetry.addData("Power",power);

            switch (slow) {

                case 0:
                    drive.go(-x, -y, -r); // drive speed max

                    telemetry.addData("Driving slow?", "No");
                    break;
                case 1:
                    drive.go(-x * 0.25, -y * 0.25, -r * 0.25);

                    telemetry.addData("Driving slow?", "Yes");
                    break;
                case 2: //for looping
                    slow = 0;
                    break;
            }
            telemetry.update();
        }
        ShooterMotor.setPower(0);
        stopMotors();
    }
}


