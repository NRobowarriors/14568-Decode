package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.pedroPathing.ShootEnum.Shooting;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueWall")
public class BlueWall extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private DcMotorEx intakeMotor, firearmMotor, firearmMotor1;
    private CRServo transfer1, transfer2, transfer3;
    private Servo fireServo;
    private Poses poses;
    private ShootEnum shootingState;
    private DriveEnum driveState;
    private boolean firstPath = true;
    private int driveIndex = 0;
    private ElapsedTime shootingTimer;




    public void runOpMode()
    {

        telemetry.addData("Status", "Initialized");


        transfer1 = hardwareMap.get(CRServo.class, "transfer1");
        transfer2 = hardwareMap.get(CRServo.class, "transfer2");
        transfer3 = hardwareMap.get(CRServo.class, "transfer3");
        fireServo = hardwareMap.get(Servo.class, "FireServo");
        transfer1.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer2.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer3.setDirection(DcMotorSimple.Direction.FORWARD);

        firearmMotor = hardwareMap.get(DcMotorEx.class, "firearmMotor");
        firearmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        firearmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        firearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firearmMotor1 = hardwareMap.get(DcMotorEx.class, "firearmMotor1");
        firearmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        firearmMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        firearmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        firearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firearmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firearmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");

        follower = Constants.createFollower(hardwareMap);
        poses = new Poses(AutoEnum.BlueWall, follower);
        driveState = DriveEnum.StartDriving;
        shootingState = ShootEnum.Waiting;
        waitForStart();
        shootingTimer = new ElapsedTime();
        fireServo.setPosition(0.5);

        while(opModeIsActive() && !isStopRequested()){
            drive();
            shoot();
            telemetry.addLine("Still in Loop");
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.addData("Plan Completion", follower.getHeadingError());
            telemetry.addData("Plan Completion", follower.getTranslationalError());
            telemetry.addLine("Out of Loop");
            telemetry.update();
        }

    }
    private void drive(){
        switch(driveState){
            case Waiting:
                break;
            case StartDriving:
                if (firstPath) {
                    follower.followPath(poses.pathPlus[driveIndex].path);
                    firstPath = false;
                }
                else {
                    follower.followPath(poses.pathPlus[driveIndex].pathChain);
                }
                follower.update();
                intakeMotor.setPower(1);
                driveState = DriveEnum.IsDriving;
                break;
            case IsDriving:
                if (!follower.isBusy()) {
                    if (poses.pathPlus[driveIndex].continueDriving) {
                        if (driveIndex == poses.pathPlus.length-1){
                            driveState = DriveEnum.Waiting;
                        }
                        else {
                            driveState = DriveEnum.StartDriving;
                        }
                    }
                    else {
                        driveState = DriveEnum.Waiting;
                        shootingState = Shooting;
                    }
                    driveIndex++;
                }
                else {
                    follower.update();
                }
                break;


        }
    }
    private void shoot(){
        switch (shootingState){
            case Waiting:
                break;
            case Shooting:
                firearmMotor.setVelocity(calcVelocity(4000));
                firearmMotor1.setVelocity(calcVelocity(4000));
                shootingState = ShootEnum.SpinUp;
                shootingTimer.reset();
                break;
            case SpinUp:
                if (shootingTimer.seconds() > 0.4){
                    transfer1.setPower(1);
                    transfer2.setPower(1);
                    transfer3.setPower(1);
                    shootingState = ShootEnum.FlickerTimer;
                    shootingTimer.reset();
                }
                break;
            case FlickerTimer:
                if (shootingTimer.seconds() > 2.1) {
                    shootingState = ShootEnum.Flicker;
                }
                break;
            case Flicker:
                fireServo.setPosition(0.1);
                shootingTimer.reset();
                shootingState = ShootEnum.FlickerReturn;
                break;
            case FlickerReturn:
                if (shootingTimer.seconds() > 0.5) {
                    firearmMotor.setVelocity(calcVelocity(0));
                    firearmMotor1.setVelocity(calcVelocity(0));
                    transfer1.setPower(0);
                    transfer2.setPower(0);
                    transfer3.setPower(0);
                    fireServo.setPosition(0.5);
                    shootingState = ShootEnum.Waiting;
                    driveState = DriveEnum.StartDriving;
                }
                break;



        }
    }
    private void shooterVelocity(int wantedVelocity) {
        firearmMotor.setVelocity(calcVelocity(wantedVelocity));
        firearmMotor1.setVelocity(calcVelocity(wantedVelocity));
    }

    private void shootCycle() {
        fireServo.setPosition(0.5);
        sleep(1000);
        fireServo.setPosition(0);
        sleep(500);
    }

    private void runFeed(int feedPower, int intakePower) {
        transfer1.setPower(feedPower);
        transfer2.setPower(feedPower);
        transfer3.setPower(feedPower);
        intakeMotor.setPower(intakePower);
    }
    public double calcVelocity(double wantedVelocity){
        return (wantedVelocity * 28) / 60;
    }
}
