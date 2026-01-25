package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class PathPlus {
    public Path path;
    public boolean continueDriving;
    public PathChain pathChain;
    Boolean nearShooting;
    public PathPlus(Path pathIn, boolean continueDrivingIn, Boolean nearShooting, PathChain pathChainIn) {
        path = pathIn;
        continueDriving = continueDrivingIn;
        this.nearShooting = nearShooting;
        pathChain = pathChainIn;
    }
}
