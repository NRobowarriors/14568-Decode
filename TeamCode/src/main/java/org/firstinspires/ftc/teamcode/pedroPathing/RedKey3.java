package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.AllAutoCode;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoEnum;

@Autonomous(name = "RedKey3")

public class RedKey3 extends LinearOpMode {

    @Override
    public void runOpMode() {
        AllAutoCode paths = new AllAutoCode(AutoEnum.RedKey3, hardwareMap, this, telemetry);
        paths.runOpMode();
    }
}
