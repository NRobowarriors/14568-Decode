package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

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

@Autonomous(name = "Bluewall")
public class BlueWall extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(18, 128, Math.toRadians(145)); // Start Pose of our robot.
    private final Pose endPose = new Pose(40, 130, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 80, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private DcMotorEx intakeMotor, firearmMotor, firearmMotor1;
    private CRServo transfer1, transfer2, transfer3;
    private Servo fireServo;
    private Path blueWall;
    private PathChain  moveOffLine;




    public void buildPaths() {


        blueWall = new Path(new BezierLine(startPose, scorePose));
        blueWall.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        blueWall.setTranslationalConstraint(5);
        blueWall.setHeadingConstraint(0.8);
        blueWall.setTimeoutConstraint(100);
        blueWall.setTValueConstraint(0.8);
        blueWall.setVelocityConstraint(0.8);




        moveOffLine = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading(), 0.8)
                .build();

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
        firearmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firearmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();

        follower.followPath(blueWall, true);
        while (follower.isBusy()) {
            follower.followPath(blueWall, true);
            follower.update();
            telemetry.addLine("Still in Loop");
            telemetry.addData("Distance Remaining", blueWall.getDistanceRemaining());
            telemetry.addData("Distance Traveled", blueWall.getDistanceTraveled());
            telemetry.addData("End Heading Constraint", blueWall.getPathEndHeadingConstraint());
            telemetry.addData("End Timeout Constraint", blueWall.getPathEndTimeoutConstraint());
            telemetry.addData("End TValue Constraint", blueWall.getPathEndTValueConstraint());
            telemetry.addData("End Translational Constraint", blueWall.getPathEndTranslationalConstraint());
            telemetry.addData("Plan Completion", blueWall.getPathCompletion());
            telemetry.addData("Is Busy", follower.isBusy());
            telemetry.addData("Plan Completion", follower.getHeadingError());
            telemetry.addData("Plan Completion", follower.getTranslationalError());
            telemetry.addLine("Out of Loop");
            telemetry.update();
        }

        shooterVelocity(3500);
        sleep(3000);
        runFeed(-1, 0.5);
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

    private void runFeed(int feedPower, double intakePower) {
        transfer1.setPower(feedPower);
        transfer2.setPower(feedPower);
        transfer3.setPower(feedPower);
        intakeMotor.setPower(intakePower);
    }

    public double calcVelocity(double wantedVelocity){
        return (wantedVelocity * 28) / 60;
    }
}

