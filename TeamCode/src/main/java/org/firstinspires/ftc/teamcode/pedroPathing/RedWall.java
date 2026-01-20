package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.ShootEnum.Shooting;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedWall")
public class RedWall extends LinearOpMode {
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


        transfer1.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer2.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer3.setDirection(DcMotorSimple.Direction.FORWARD);


        firearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firearmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firearmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");

        follower = Constants.createFollower(hardwareMap);
        poses = new Poses(AutoEnum.BlueKey, follower);
        driveState = DriveEnum.StartDriving;
        shootingState = ShootEnum.Waiting;
        waitForStart();
        shootingTimer = new ElapsedTime();
        fireServo.setPosition(0.5);

        while(opModeIsActive() && !isStopRequested()){
            drive();
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




        }





