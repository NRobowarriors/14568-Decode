package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class Poses {
    public Poses (AutoEnum auto, Follower followerIn) {
        follower = followerIn;
        switch (auto)
        {
            case BlueWall:
                createPaths(BlueWallPoses);
                break;
            case RedWall:
                createPaths(RedWallPoses);
                break;
            case BlueKey:
                createPaths(BlueKeyPoses);
                break;
            case RedKey:
                createPaths(RedKeyPoses);
                break;

        }
    }
    private Follower follower;
    public Path startingPath;
    public PathChain[] PathChains;
    private Pose[] BlueWallPoses = new Pose[]{
            new Pose(18, 128, Math.toRadians(145)), //starting pose
            new Pose(60, 80, Math.toRadians(135)), //scoring pose
            new Pose(29.5, 83, Math.toRadians(0)),
            new Pose(17.5, 83, Math.toRadians(0)),
            new Pose(60, 80, Math.toRadians(135)), //scoring pose
            new Pose(29.5, 59, Math.toRadians(0)),
            new Pose(17.5, 59, Math.toRadians(0)),
            new Pose(60, 80, Math.toRadians(135)), //scoring pose
            new Pose(29.5, 35, Math.toRadians(0)),
            new Pose(17.5, 35, Math.toRadians(0)),
            new Pose(60, 80, Math.toRadians(135)), //scoring pose
            new Pose(40, 130, Math.toRadians(90)) //move off line
    } ;
    private Pose[] RedWallPoses = new Pose[]{
            new Pose(120, 128, Math.toRadians(35)), //starting pose
            new Pose(78, 80, Math.toRadians(45)), //scoring pose
            new Pose(120, 83, Math.toRadians(180)),
            new Pose(132, 83, Math.toRadians(180)),
            new Pose(78, 80, Math.toRadians(45)), //scoring pose
            new Pose(120, 59, Math.toRadians(180)),
            new Pose(132, 59, Math.toRadians(180)),
            new Pose(78, 80, Math.toRadians(45)), //scoring pose
            new Pose(120, 35, Math.toRadians(180)),
            new Pose(132, 35, Math.toRadians(180)),
            new Pose(78, 80, Math.toRadians(45)), //scoring pose
            new Pose(90, 120, Math.toRadians(90)) //move off line
    } ;
    private Pose[] BlueKeyPoses = new Pose[]{
            new Pose(60, 9, Math.toRadians(90)), //starting pose
            new Pose(60, 40, Math.toRadians(110)), //scoring pose
            new Pose(29.5, 35, Math.toRadians(0)),
            new Pose(17.5, 35, Math.toRadians(0)),
            new Pose(60, 80, Math.toRadians(110)), //scoring pose
            new Pose(29.5, 59, Math.toRadians(0)),
            new Pose(17.5, 59, Math.toRadians(0)),
            new Pose(60, 80, Math.toRadians(110)), //scoring pose
            new Pose(29.5, 83, Math.toRadians(0)),
            new Pose(17.5, 85, Math.toRadians(0)),
            new Pose(60, 80, Math.toRadians(110)), //scoring pose
            new Pose(40, 130, Math.toRadians(90)) //move off line
    } ;
    private Pose[] RedKeyPoses = new Pose[]{
            new Pose(84, 9, Math.toRadians(90)), //starting pose
            new Pose(84, 10, Math.toRadians(70)), //scoring pose
            new Pose(120, 35, Math.toRadians(180)),
            new Pose(132, 35, Math.toRadians(180)),
            new Pose(84, 10, Math.toRadians(70)), //scoring pose
            new Pose(120, 59, Math.toRadians(180)),
            new Pose(132, 59, Math.toRadians(180)),
            new Pose(84, 10, Math.toRadians(70)), //scoring pose
            new Pose(120, 83, Math.toRadians(180)),
            new Pose(132, 83, Math.toRadians(180)),
            new Pose(84, 10, Math.toRadians(70)), //scoring pose
            new Pose(84, 40, Math.toRadians(90)) //move off line
    } ;

    private void createPaths(Pose[] poses){
        follower.setStartingPose(poses[0]);
        startingPath = new Path(new BezierLine(poses[0], poses[1]));
        startingPath.setLinearHeadingInterpolation(poses[0].getHeading(), poses[1].getHeading());

        PathChains = new PathChain[poses.length-2];
        for (int i = 1; i <= poses.length-2; i++)
        {
            PathChains[i-1] = follower.pathBuilder()
                        .addPath(new BezierLine(poses[i], poses[i+1]))
                        .setLinearHeadingInterpolation(poses[i].getHeading(), poses[i+1].getHeading(), 0.8)
                        .build();
        };
    }
}

