package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.pedroPathing.ShootEnum.Shooting;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "RedKey")
public class RedKey extends LinearOpMode {
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
        transfer1.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer2.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer3.setDirection(DcMotorSimple.Direction.REVERSE);

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
        poses = new Poses(AutoEnum.RedKey, follower);

        waitForStart();

        while(opModeIsActive()){
            drive();
            shoot();
        }


        follower.followPath(poses.startingPath, true);

        while (follower.isBusy()) {
            follower.update();
            telemetry.addLine("Still in Loop");
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.addData("Plan Completion", follower.getHeadingError());
            telemetry.addData("Plan Completion", follower.getTranslationalError());
            telemetry.addLine("Out of Loop");
            telemetry.update();
        }

        shooterVelocity(4750);
        sleep(2000);
        shootCycle();
        sleep(1000);
        runFeed(-1, 1);
        sleep(1000);
        shootCycle();
        sleep(300);
        shootCycle();
        sleep(300);
        shootCycle();
        shooterVelocity(0);
        runFeed(0, 0);


        follower.followPath(moveOffLine, true);
        while (follower.isBusy()) {
            follower.followPath(moveOffLine, true);
            follower.update();
        }

    }
    private void drive(){
        switch(driveState){
            case Waiting:
                break;
            case StartDriving:
                if (firstPath) follower.followPath(poses.startingPath);
                else {
                    follower.followPath(poses.PathChains[driveIndex]);
                    driveIndex++;
                }
                follower.update();
                driveState = DriveEnum.IsDriving;
                break;
            case IsDriving:
                if (!follower.isBusy()) {
                    driveState = DriveEnum.Waiting;
                    shootingState = Shooting;
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
               firearmMotor.setVelocity(calcVelocity(4750));
               firearmMotor1.setVelocity(calcVelocity(4750));
               transfer1.setPower(1);
               transfer2.setPower(1);
               transfer3.setPower(1);
               shootingState = ShootEnum.FlickerTimer;
               shootingTimer.reset();
               break;
            case FlickerTimer:
                if (shootingTimer.seconds() > 1) {
                    shootingState = ShootEnum.Flicker;
                }
                break;
            case Flicker:
                fireServo.setPosition(0.5);
                shootingTimer.reset();
                shootingState = ShootEnum.FlickerTimer;
                break;
            case FlickerReturn:
                if (shootingTimer.seconds() > 0.2) {
                    shootingState = ShootEnum.Waiting;
                    fireServo.setPosition(0);
                }



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
