package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.AllAutoCode;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoEnum;

@Autonomous(name = "BlueKey")

public class BlueKey extends LinearOpMode {

    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.BlueKey, hardwareMap, this, telemetry);
        paths.runOpMode();
    }
}
