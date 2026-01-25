package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
public class PosePlus {
  public Pose pose;
  public boolean continueDriving;
  Boolean nearShooting;
  public PosePlus(Pose poseIn, boolean continueDrivingIn, Boolean nearShooting){
      pose = poseIn;
      continueDriving = continueDrivingIn;
      this.nearShooting = nearShooting;
  }
}
