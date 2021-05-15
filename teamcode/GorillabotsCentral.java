package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.RobotHardware;
import org.firstinspires.ftc.teamcode.Components.VisionPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.AutoDrive;
import org.firstinspires.ftc.teamcode.Components.MecanumDrive;
import org.firstinspires.ftc.teamcode.Components.RevGyro;
import org.firstinspires.ftc.teamcode.Components.Sensors;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Components.Grabber;

import java.security.Guard;

import static java.lang.Math.abs;

public abstract class GorillabotsCentral extends LinearOpMode {

    public AutoDrive ADrive;
    public Sensors sensors;
    public MecanumDrive drive;
    public RevGyro gyro;
    public ElapsedTime timer;
    public VisionPipeline pipeline;
    public OpenCvCamera webcam;
    public RobotHardware robot;
   // public Grabber grabber;

    /*
    HUB # 1 (says hub 2 on the phone but 1 in real life)

    Motors:
    0: mfr
    1: mbr

    Servos:
    0: rotate
    1: rollerF
    2: capstone
    3: hookR
    4: rollerB

    I2C:
    0: imu1
    1: rangeF
    2: rangeB
    3: rangeR

    HUB # 2 (says hub 1 on the phone but 2 in real life)

    Motors:
    0: parker
    1: lift
    2: mfl
    3: mbl

    Servos:
    4: hookL

    I2C:
    0: imu
    1: rangeL

    Digital Devices:
    3: liftBot
     */

    public void initializeComponents() {
        timer = new ElapsedTime();

        ADrive = new AutoDrive(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);

        sensors = new Sensors(hardwareMap, telemetry);

        gyro = new RevGyro(hardwareMap, telemetry);

        robot = new RobotHardware(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new VisionPipeline();
        webcam.setPipeline(pipeline);

        telemetry.addData("done:", "init");

    }

    public void initializeComponentsAutonomous() {


        ADrive = new AutoDrive(hardwareMap, telemetry);

        drive = new MecanumDrive(hardwareMap, telemetry);

        sensors = new Sensors(hardwareMap, telemetry);

        gyro = new RevGyro(hardwareMap, telemetry);

        robot = new RobotHardware(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new VisionPipeline();
        webcam.setPipeline(pipeline);

        robot.Guard(RobotHardware.guardUp);

        telemetry.addData("done:", "init");
        telemetry.update();
    }
    public void dropWobble(){
        robot.WobbleArm.setPower(0.5);
        sleep(500);
    }
    public void liftWobble(){
        robot.WobbleArm.setPower(-0.5);
        sleep(500);
    }
    public void moveTransfer(){
        robot.Transfer.setPower(0.9);
    }
    public void stopTransfer(){
        robot.Transfer.setPower(0);
    }
    public void reverseTransfer(){
        robot.Transfer.setPower(-0.4);
    }

    public void rampShooter(double pwr){
        robot.ShooterMotor.setPower(pwr);
    }
    public void stopShooter(){
        robot.ShooterMotor.setPower(0);
    }

    public void startIntake(){
        robot.Intake.setPower(-0.8);
    }
    public void reverseIntake(){
        robot.Intake.setPower(0.5);
    }
    public void stopIntake(){
        robot.Intake.setPower(0);
    }

    public void shootAuto(double pwr)
    {
        robot.ShooterMotor.setPower(pwr);
        sleep(1500);//2000
        moveTransfer();
        //sleep(500);
        startIntake();
        sleep(100);
        stopTransfer();
        stopIntake();
        sleep(1000);
        moveTransfer();
        startIntake();
        sleep(90);
        stopTransfer();
        stopIntake();
        sleep(1000);
        startIntake();
        moveTransfer();
        sleep(400);
        stopIntake();
        stopTransfer();
        sleep(200);
        stopShooter();
    }

    public void shootAuto2(double pwr)
    {
        robot.ShooterMotor.setPower(pwr);
        MoveTo(180, .6);
        moveTransfer();
        startIntake();
        sleep(2000);
        stopIntake();
        stopTransfer();
        sleep(200);
        stopShooter();
    }
    public void runTrack0(){
        DcMotor Intake;
        Intake = hardwareMap.dcMotor.get("Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor ShooterMotor;
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor Transfer;
        Transfer = hardwareMap.dcMotor.get("Transfer");
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Servo grabber;
       grabber = hardwareMap.get(Servo.class, "grabber");
        final double Grabber_Close = .6;
        final double Grabber_Open = .05;
        DcMotor WobbleArm;
        WobbleArm = hardwareMap.dcMotor.get("WobbleArm");
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MoveUntilTime(1270, 182, 0.6);//1430      189
        stopMotors();
        sleep(200);
        TurnAbsolute(-12.7, .3, .6);//-15.3    -9.7
       shootAuto(0.73);
        MoveUntilTime(545,180,0.5);//685
        sleep(300);
        TurnAbsolute(270,.3,.5);
        sleep(300);
        MoveUntilTime(800,90,0.5);
        dropWobble();
        sleep(600);
        grabber.setPosition(Grabber_Open);
        sleep(1000);
        MoveUntilTime(500,270,0.5);
        sleep(200);
        grabber.setPosition(Grabber_Close);
        liftWobble();
        stopMotors();
    }

    public void runTrack1(){
        DcMotor Intake;
        Intake = hardwareMap.dcMotor.get("Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor ShooterMotor;
        ShooterMotor = hardwareMap.dcMotor.get("ShooterMotor");
        ShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor Transfer;
        Transfer = hardwareMap.dcMotor.get("Transfer");
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Servo grabber;
        grabber = hardwareMap.get(Servo.class, "grabber");
        final double Grabber_Close = .6;
        final double Grabber_Open = .05;
        DcMotor WobbleArm;
        WobbleArm = hardwareMap.dcMotor.get("WobbleArm");
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // MoveUntilTime(1390, 140, 0.6);//1430      189
        MoveUntilEncoder(100,140,.4);
        stopMotors();
        robot.Guard(RobotHardware.guardDown);
        sleep(200);
        TurnAbsolute(-22, .3, .6);//-15.8
        sleep(200);
        stopMotors();
        shootAuto(0.608);
        TurnAbsolute(16.4,.3,.6);//10.8
        sleep(200);
        MoveUntilTime(1350,200,0.5);
        sleep(300);
        TurnAbsolute(-6,.3,.6);
        MoveUntilRangeR(10,270,0.45);
        //MoveUntilTime(2500,270,0.6);
        sleep(300);
        dropWobble();
        sleep(700);
        grabber.setPosition(Grabber_Open);
        sleep(400);
        MoveUntilTime(500,270,0.4);
        sleep(200);
        grabber.setPosition(Grabber_Close);
        liftWobble();
        sleep(200);
        TurnAbsolute(-8.4,.3,.6);
        MoveUntilTime(700,0,.55);
        sleep(200);
        startIntake();
        moveTransfer();
        TurnAbsolute(-13.7,.3,.6);
        sleep(300);
        MoveUntilTime(1330,42,0.49);
        sleep(200);
        MoveUntilTime(300,0,0.45);
        sleep(500);
        stopIntake();
        reverseTransfer();
        MoveUntilTime(800,190,0.56);
        sleep(200);
        TurnAbsolute(2.5,.3,.6);//+4.5
        sleep(600);
        stopTransfer();
        sleep(300);
        rampShooter(0.65);
        sleep(2100);
        moveTransfer();
        startIntake();
        sleep(1500);
        stopShooter();
        stopTransfer();
        sleep(500);
        MoveUntilTime(540,180,.6);

        stopMotors();
    }
    public void runTrack4(){
        Servo grabber;
        grabber = hardwareMap.get(Servo.class, "grabber");
        final double Grabber_Close = .6;
        final double Grabber_Open = .05;
        DcMotor WobbleArm;
        WobbleArm = hardwareMap.dcMotor.get("WobbleArm");
        WobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // gyro.resetAngle();
        MoveUntilTime(1390, 140, 0.6);//1430      189
        stopMotors();
        robot.Guard(RobotHardware.guardDown);
        sleep(200);
        TurnAbsolute(-22, .3, .6);//-15.8
        sleep(200);
        stopMotors();
        shootAuto(0.608);
        TurnAbsolute(16.4,.3,.6);//10.8
        sleep(200);
        MoveUntilTime(699,200,1);//700
        sleep(300);
        MoveUntilTime(1140,285,1);
        sleep(300);
        TurnAbsolute(-86,.3,.6);
        sleep(200);
        MoveUntilTime(555,90,0.4);
        sleep(200);
        MoveUntilRangeF(7,180,0.3);
        //MoveUntilTime(1650,140,.28);
        sleep(300);
        dropWobble();
        sleep(700);
        grabber.setPosition(Grabber_Open);
        sleep(400);
        MoveUntilTime(500,270,0.4);
        sleep(200);
        liftWobble();
        sleep(200);
        MoveUntilTime(525,0,1);
        //MoveBackUntilRangeF(37.5,0,0.23);
        sleep(200);
        robot.Guard(RobotHardware.guardDown);
        TurnAbsolute(0,.3,.6);
        sleep(300);
        MoveUntilTime(430,0,1);
        sleep(300);
        //MoveLeftUntilRangeRG(34,90,.34,180);
        startIntake();
        moveTransfer();
        rampShooter(-0.3);
        MoveUntilTime(3560,0,.25);//3000 2660
        sleep(1300);
        reverseTransfer();
        startIntake();
        rampShooter(-0.2);
        MoveUntilTime(900,170,0.5);
        TurnAbsolute(-12.5,.3,.6);
        sleep(1000);
        stopTransfer();
       shootAuto(0.65);
       // MoveUntilTime(720,180,1);

        stopMotors();
    }

    public static final int degreeCorrection = 180; //180
    //160
    public static final double COUNTS_PER_MOTOR_REV = 384;     //12.5:1
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void startVisionProcessing() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public void stopVisionProcessing(){
        webcam.stopStreaming();
    }

    public void MoveUntilEncoder(double distance, double degree, double power) {

        drive.mfr.setDirection(DcMotor.Direction.REVERSE);
        drive.mfl.setDirection(DcMotor.Direction.FORWARD);
        drive.mbr.setDirection(DcMotor.Direction.REVERSE);
        drive.mbl.setDirection(DcMotor.Direction.FORWARD);

        double degreeRad = Math.toRadians(degree - degreeCorrection);
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        setDriveEncoderOn(true);

        int rightFrontStartPos = drive.mfr.getCurrentPosition();
        int rightRearStartPos = drive.mbr.getCurrentPosition();
        int leftFrontStartPos = drive.mfl.getCurrentPosition();
        int leftRearStartPos = drive.mbl.getCurrentPosition();

        int target = (int) (distance * COUNTS_PER_INCH);

        int rightFrontEndPos = rightFrontStartPos + (int) (target * (-sn + cs));
        int leftFrontEndPos = leftFrontStartPos + (int) (target * (sn + cs));
        int rightRearEndPos = rightRearStartPos + (int) (target * (sn + cs));
        int leftRearEndPos = leftRearStartPos + (int) (target * (-sn + cs));

        double pwr = power;

        double rightFrontPower = pwr * (-sn + cs);
        double leftFrontPower = pwr * (sn + cs);
        double rightRearPower = pwr * (sn + cs);
        double leftRearPower = pwr * (-sn + cs);


        drive.mfr.setPower(rightFrontPower);
        drive.mfl.setPower(leftFrontPower);
        drive.mbr.setPower(rightRearPower);
        drive.mbl.setPower(leftRearPower);

        drive.mfr.setTargetPosition(rightFrontEndPos);
        drive.mfl.setTargetPosition(leftFrontEndPos);
        drive.mbr.setTargetPosition(rightRearEndPos);
        drive.mbl.setTargetPosition(leftRearEndPos);


        while (drive.mfl.isBusy() && opModeIsActive()) {
        }
        /*|| mfl.isBusy() || mbr.isBusy() || mbl.isBusy())*/
        stopMotors();
    }

    public void MoveUntilEncoderTest(double distance, double degree, double power) {

        drive.mfr.setDirection(DcMotor.Direction.REVERSE);
        drive.mfl.setDirection(DcMotor.Direction.FORWARD);
        drive.mbr.setDirection(DcMotor.Direction.REVERSE);
        drive.mbl.setDirection(DcMotor.Direction.FORWARD);

        double degreeRad = Math.toRadians(degree - degreeCorrection);
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        setDriveEncoderOn(true);
        drive.resetDrivenDistance();
        int rightFrontStartPos = drive.mfr.getCurrentPosition();
        int rightRearStartPos = drive.mbr.getCurrentPosition();
        int leftFrontStartPos = drive.mfl.getCurrentPosition();
        int leftRearStartPos = drive.mbl.getCurrentPosition();

        int target = (int) (distance * COUNTS_PER_INCH);

        int rightFrontEndPos = rightFrontStartPos + (int) (target * (-sn + cs));
        int leftFrontEndPos = leftFrontStartPos + (int) (target * (sn + cs));
        int rightRearEndPos = rightRearStartPos + (int) (target * (sn + cs));
        int leftRearEndPos = leftRearStartPos + (int) (target * (-sn + cs));

        double pwr = power;

        double rightFrontPower = pwr * (-sn + cs);
        double leftFrontPower = pwr * (sn + cs);
        double rightRearPower = pwr * (sn + cs);
        double leftRearPower = pwr * (-sn + cs);

       while(drive.mfr.getCurrentPosition()<leftFrontEndPos){ drive.mfr.setPower(rightFrontPower);
        drive.mfl.setPower(leftFrontPower);
        drive.mbr.setPower(rightRearPower);
        drive.mbl.setPower(leftRearPower);

        drive.mfr.setTargetPosition(rightFrontEndPos);
        drive.mfl.setTargetPosition(leftFrontEndPos);
        drive.mbr.setTargetPosition(rightRearEndPos);
        drive.mbl.setTargetPosition(leftRearEndPos);}

        stopMotors();
        sleep(600);
        if(drive.mfr.getCurrentPosition() > rightFrontEndPos+1 || drive.mfr.getCurrentPosition()< rightFrontEndPos-1){
            drive.mbr.setPower(0.2);
            drive.mfr.setPower(0.2);
            drive.mfr.setTargetPosition(rightFrontEndPos-drive.mfr.getCurrentPosition());
            drive.mbr.setTargetPosition(rightRearEndPos-drive.mbr.getCurrentPosition());
        }

        /*if(drive.mfr.getCurrentPosition() > rightFrontEndPos+1 || drive.mfr.getCurrentPosition()< rightFrontEndPos-1){
            drive.mbr.setPower(0.2);
            drive.mfr.setPower(0.2);
            drive.mfr.setTargetPosition(rightFrontEndPos-drive.mfr.getCurrentPosition());
            drive.mbr.setTargetPosition(rightRearEndPos-drive.mbr.getCurrentPosition());
        }
        while (drive.mfl.isBusy() && opModeIsActive()) {
        }
        /*|| mfl.isBusy() || mbr.isBusy() || mbl.isBusy())*/
        stopMotors();
    }

    public void MoveUntilEncoder2(double distance, double degree, double power) {

        drive.mfr.setDirection(DcMotor.Direction.REVERSE);
        drive.mfl.setDirection(DcMotor.Direction.FORWARD);
        drive.mbr.setDirection(DcMotor.Direction.REVERSE);
        drive.mbl.setDirection(DcMotor.Direction.FORWARD);

        double degreeRad = Math.toRadians(degree - degreeCorrection);
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        setDriveEncoderOn(true);

        int rightFrontStartPos = drive.mfr.getCurrentPosition();
        int rightRearStartPos = drive.mbr.getCurrentPosition();
        int leftFrontStartPos = drive.mfl.getCurrentPosition();
        int leftRearStartPos = drive.mbl.getCurrentPosition();

        int target = (int) (distance * COUNTS_PER_INCH);

        int rightFrontEndPos = rightFrontStartPos + (int) (target * (-sn + cs));
        int leftFrontEndPos = leftFrontStartPos + (int) (target * (sn + cs));
        int rightRearEndPos = rightRearStartPos + (int) (target * (sn + cs));
        int leftRearEndPos = leftRearStartPos + (int) (target * (-sn + cs));

        double pwr = power;

        double rightFrontPower = pwr * (-sn + cs);
        double leftFrontPower = pwr * (sn + cs);
        double rightRearPower = pwr * (sn + cs);
        double leftRearPower = pwr * (-sn + cs);

        drive.mfr.setPower(rightFrontPower);
        drive.mfl.setPower(leftFrontPower);
        drive.mbr.setPower(rightRearPower);
        drive.mbl.setPower(leftRearPower);

        drive.mfr.setTargetPosition(rightFrontEndPos);
        drive.mfl.setTargetPosition(leftFrontEndPos);
        drive.mbr.setTargetPosition(rightRearEndPos);
        drive.mbl.setTargetPosition(leftRearEndPos);

        while ((drive.mfr.isBusy() || drive.mfl.isBusy()) && opModeIsActive()) {
        }
        /*|| mfl.isBusy() || mbr.isBusy() || mbl.isBusy())*/
        stopMotors();
    }

    public void MoveUntilRangeF(double distance, double direction, double power) {
        setDriveEncoderOn(false);
        setMotorsBackwards();
        MoveTo(direction, power);
        while ((sensors.getDistanceF() > distance) && opModeIsActive()) {
            MoveTo(direction, power);
            telemetry.addData("d", sensors.getDistanceF());
            telemetry.update();
        }
        stopMotors();
    }
    public void MoveBackUntilRangeF(double distance, double direction, double power) {
        setDriveEncoderOn(false);
        setMotorsBackwards();
        MoveTo(direction, power);
        while ((sensors.getDistanceF() < distance) && opModeIsActive()) {
            MoveTo(direction, power);
            telemetry.addData("d", sensors.getDistanceF());
            telemetry.update();
        }
        stopMotors();
    }

   // public void MoveUntilRangeB(double distance, double direction, double power) {
     //   setDriveEncoderOn(false);
       // setMotorsBackwards();
        //MoveTo(direction, power);
        //while ((sensors.getDistanceB() > distance) && opModeIsActive()) {
          //  MoveTo(direction, power);
        //}
        //stopMotors();
    //}



    public void MoveUntilRangeFwithG(double distance, double direction, double power,double gyroT) {
        setDriveEncoderOn(false);
        setMotorsBackwards();
        MoveTo(direction, power);
        while ((sensors.getDistanceF() > distance) && opModeIsActive()) {
            MoveTowR(direction, power, (gyro.getAngle() - gyroT) / 50);
            telemetry.addData("f", sensors.getDistanceF());
            telemetry.update();
        }
        stopMotors();
    }



    public void MoveUntilRangeRG(double distance, double direction, double power, double gyroT) {
        setDriveEncoderOn(false);
        setMotorsBackwards();
        MoveTo(direction, power);
        while (abs(sensors.getDistanceR() - distance) > .2 && opModeIsActive()) {
            if(gamepad1.a){
                distance = 0;
            }
            MoveTowR(direction, power, (gyro.getAngle() - gyroT) / 50);
            telemetry.addData("d", sensors.getDistanceR());
            telemetry.update();
        }
        stopMotors();
    }
    public void MoveUntilRangeR(double distance, double direction, double power) {
        setDriveEncoderOn(false);
        setMotorsBackwards();
        //MoveTo(direction, power);
        while (sensors.getDistanceR() - distance > 2 && opModeIsActive()) {
            MoveTo(direction, power);
            telemetry.addData("d", sensors.getDistanceR());
            telemetry.update();
        }
        stopMotors();
    }
    public void MoveLeftUntilRangeRG(double distance, double direction, double power, double gyroT) {
        setDriveEncoderOn(false);
        setMotorsBackwards();
        MoveTo(direction, power);
        while (sensors.getDistanceR() - distance < 2 && opModeIsActive()) {
            if(gamepad1.a){
                distance = 0;
            }
            MoveTo(direction, power);
            telemetry.addData("d", sensors.getDistanceR());
            telemetry.update();
        }
        stopMotors();
    }
    /*public void MoveUntilRangeLG(double distance, double direction, double power, double gyroT) {
        setDriveEncoderOn(false);
        setMotorsBackwards();
        MoveTo(direction, power);
        while ((sensors.getDistanceL() > distance) && opModeIsActive())  {
            if(gamepad1.a){
                distance = 0;
            }
            MoveTowR(direction, power, (gyro.getAngle() - gyroT) / 50);
        }
        stopMotors();
    }*/

    public void MoveUntilEncoderGYRO(double distance, double direction, double power, double gyroT) {
        setMotorsBackwards();
        setDriveEncoderOn(false); //false
        int initPos = drive.mfr.getCurrentPosition();
        MoveTo(direction, power);
        distance = distance * 1000 / 34;
        while ((abs(drive.mfr.getCurrentPosition() - initPos) < abs(distance)) && opModeIsActive()) {
            MoveTowR(direction, power, (gyro.getAngle() - gyroT) / 50);
        }
        stopMotors();
    }
    public void MoveUntilEncoderGYROtest1(double distance, double direction, double power, double gyroT) {
        setDriveEncoderOn(false); //false
        int initPos = drive.mfr.getCurrentPosition();
        gyro.resetAngle();
        MoveTo(direction, power);
        distance = distance * 1000 / 34;
        while ((abs(drive.mfr.getCurrentPosition() - initPos) < abs(distance)) && opModeIsActive()) {
            MoveTowR((gyro.getAngle() - gyroT), power, (gyro.getAngle() - gyroT));
        }
        stopMotors();
    }

    public void MoveUntilEncoderGYRORangeR(double distance, double direction, double power, double gyroT, double rangeT) {
        setMotorsBackwards();
        setDriveEncoderOn(false);
        int initPos = drive.mfr.getCurrentPosition();
        double correctionDirection = 0;
        MoveTo(direction, power);
        distance = distance * 1000 / 34;
        while ((abs(drive.mfr.getCurrentPosition() - initPos) < abs(distance)) && opModeIsActive()) {
            telemetry.addData("getCurPos", drive.mfr.getCurrentPosition());
            telemetry.addData("s", drive.mfr.getCurrentPosition() - initPos);
            telemetry.addData("range",correctionDirection);
            telemetry.update();
            correctionDirection = (sensors.getDistanceR() - rangeT) * 5;
            MoveTowR(direction + correctionDirection, power, (gyro.getAngle() - gyroT) / 50);
        }
        stopMotors();
    }

    public void MoveUntilTime(long timeMilli, double direction, double power) {
        setMotorsBackwards();
        setDriveEncoderOn(false);
        MoveTo(direction, power);
        sleep(timeMilli);
        stopMotors();
    }

    public void MoveTo(double degree, double power) {
        double degreeRad = Math.toRadians(degree - degreeCorrection); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        double fr = power * (-sn + cs);
        double fl = power * (sn + cs);
        double br = power * (sn + cs);
        double bl = power * (-sn + cs);

        drive.mfl.setPower(fl);
        drive.mfr.setPower(fr);
        drive.mbl.setPower(bl);
        drive.mbr.setPower(br);
    }

    public void MoveTowR(double degree, double power, double r) {
        double degreeRad = Math.toRadians(degree - degreeCorrection); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        double fr = Range.clip((power * (-sn + cs)) + r, -1, 1);
        double fl = Range.clip((power * (sn + cs)) - r, -1, 1);
        double br = Range.clip((power * (sn + cs)) + r, -1, 1);
        double bl = Range.clip((power * (-sn + cs)) - r, -1, 1);


        drive.mfl.setPower(fl);
        drive.mfr.setPower(fr);
        drive.mbl.setPower(bl);
        drive.mbr.setPower(br);
    }
    public void MoveTowRtime(double degree, double power, double r, long time) {
        double degreeRad = Math.toRadians(degree - degreeCorrection); // Convert to radians
        double cs = Math.cos(degreeRad);
        double sn = Math.sin(degreeRad);

        double fr = Range.clip((power * (-sn + cs)) + r, -.6, .6);
        double fl = Range.clip((power * (sn + cs)) - r, -.6, .6);
        double br = Range.clip((power * (sn + cs)) + r, -.6, .6);
        double bl = Range.clip((power * (-sn + cs)) - r, -.6, .6);


        drive.mfl.setPower(fl);
        drive.mfr.setPower(fr);
        drive.mbl.setPower(bl);
        drive.mbr.setPower(br);
        sleep(time);
        stopMotors();
    }

    public void stopMotors() {
        drive.mfr.setPower(0);
        drive.mfl.setPower(0);
        drive.mbr.setPower(0);
        drive.mbl.setPower(0);
    }

    public void setMotorsBackwards() {
        drive.mfr.setDirection(DcMotor.Direction.REVERSE);
        drive.mfl.setDirection(DcMotor.Direction.FORWARD);
        drive.mbr.setDirection(DcMotor.Direction.REVERSE);
        drive.mbl.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setDriveEncoderOn(boolean on) {
        if (on) {
            drive.mfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.mfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.mfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive.mbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.mbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.mbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive.mfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.mfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.mfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive.mbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.mbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.mbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            drive.mfr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.mbr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.mfl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.mbl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void TurnAbsolute(double TargetDegree, double min, double max) {
        // clock is negative; anti-clock positive degree
        // rotate range is (-90,90)

        setMotorsBackwards();

        if (TargetDegree > 180) {
            TargetDegree = 180;
        }
        if (TargetDegree < -180) {
            TargetDegree = -180;
        }

        double MaxPower = max;
        double minPower = min;

        double correctionDegree = 5;
        double beginDegree;
        double currentDegree;

        double target;
        double angleDiff;
        double maxTime = 6; //seconds
        ElapsedTime runtime = new ElapsedTime();

        setDriveEncoderOn(false);

        beginDegree = gyro.getAngle();

        runtime.reset();
        runtime.startTime();

        angleDiff = TargetDegree - beginDegree;
        while (abs(angleDiff) > 1 && runtime.seconds() < maxTime && opModeIsActive()) {
            double leftPower;
            double rightPower;
            currentDegree = gyro.getAngle();
            angleDiff = TargetDegree - currentDegree;
            if (angleDiff > 180) {
                angleDiff = angleDiff - 360;
            }
            if (angleDiff < -180) {
                angleDiff = angleDiff + 360;
            }

            if (angleDiff < 0) {
                angleDiff = angleDiff + correctionDegree;
            }
            if (angleDiff > 0) {
                angleDiff = angleDiff - correctionDegree;
            }

            double drivea;
            drivea = (angleDiff) / 100.0;

            if (abs(drivea) > MaxPower) {
                drivea = MaxPower * abs(drivea) / drivea;
            }
            if (abs(drivea) < minPower) {
                if (drivea > 0) {
                    drivea = minPower;
                } else if (drivea < 0) {
                    drivea = -minPower;
                } else {
                    drivea = 0;
                }
            }

            leftPower = Range.clip(-drivea, -1.0, 1.0);
            rightPower = Range.clip(drivea, -1.0, 1.0);

            drive.mfl.setPower(-rightPower);
            drive.mbl.setPower(-rightPower);
            drive.mfr.setPower(-leftPower);
            drive.mbr.setPower(-leftPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("right Power", rightPower);
            telemetry.addData("beginDegree", beginDegree);
            telemetry.addData("CurrentDegree", currentDegree);
            telemetry.addData("angleDiff", angleDiff);
            telemetry.update();
        }
        stopMotors();

        telemetry.addData("Current ZDegree", gyro.getAngle());
        telemetry.update();
    }


    public int getRings() {
        int rings = 4;
        while (!isStarted() && !isStopRequested()) {
            rings = pipeline.getPos();
            telemetry.addData("Rings", rings);
            telemetry.update();
        }
        return rings;
    }

}