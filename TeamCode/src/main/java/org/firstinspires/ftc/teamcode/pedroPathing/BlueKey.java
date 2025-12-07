package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

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

@Autonomous(name = "BlueKey")
public class BlueKey extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(60, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose endPose = new Pose(60, 40, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 11, Math.toRadians(110)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private DcMotorEx intakeMotor, firearmMotor, firearmMotor1;
    private CRServo transfer1, transfer2, transfer3;
    private Servo fireServo;
    private Path blueKey;
    private PathChain  moveOffLine;




    public void buildPaths() {


        blueKey = new Path(new BezierLine(startPose, scorePose));
        blueKey.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        blueKey.setTranslationalConstraint(5);
        blueKey.setHeadingConstraint(0.9);
        blueKey.setTimeoutConstraint(100);
        blueKey.setTValueConstraint(0.9);
        blueKey.setVelocityConstraint(0.9);


      //  blueKey = new Path(new BezierLine(scorePose, endPose));
      //  blueKey.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());

        moveOffLine = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), 0.8)
                .build();
    //    moveOffLine = follower.pathBuilder()
             //   .addPath(new BezierLine(scorePose, endPose))
               // .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
               // .build();

     /*   scorePickup1 = follower.pathBuilder()//only drives rn no shoot yet
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
        */       // .build();
    }

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
        firearmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();

        follower.followPath(blueKey, true);

        while (follower.isBusy()) {
            follower.followPath(blueKey, true);
            follower.update();
            telemetry.addLine("Still in Loop");
            telemetry.addData("Distance Remaining", blueKey.getDistanceRemaining());
            telemetry.addData("Distance Traveled", blueKey.getDistanceTraveled());
            telemetry.addData("End Heading Constraint", blueKey.getPathEndHeadingConstraint());
            telemetry.addData("End Timeout Constraint", blueKey.getPathEndTimeoutConstraint());
            telemetry.addData("End TValue Constraint", blueKey.getPathEndTValueConstraint());
            telemetry.addData("End Translational Constraint", blueKey.getPathEndTranslationalConstraint());
            telemetry.addData("Plan Completion", blueKey.getPathCompletion());
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.addData("Plan Completion", follower.getHeadingError());
            telemetry.addData("Plan Completion", follower.getTranslationalError());
            telemetry.addLine("Out of Loop");
            telemetry.update();
        }

        shooterVelocity(4750);
        sleep(3000);
        runFeed(-1, 1);
        sleep(1000);
        shootCycle();
        shootCycle();
        shootCycle();
        shooterVelocity(0);
        runFeed(0, 0);

        do {
            follower.followPath(moveOffLine, true);
            follower.update();
        }
        while (follower.isBusy());
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

