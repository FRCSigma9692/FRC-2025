package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlideSub;

public class GlideCmd extends Command {

 // LaserCan lc;

  GlideSub n;
  double s;
  boolean flag;

  public GlideCmd(GlideSub n, double position) {
    this.n = n;
    this.s = position;
    addRequirements(n);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
   n.glide(s);
    //n.drop(s);
  }

  @Override
  public void end(boolean interrupted) {
    n.glide(0);
    //n.passTheCoral(0);
    //n.drop(0);
  }

  @Override
  public boolean isFinished() {
    if(n.glidemotor.getForwardLimitSwitch().isPressed() && s>=0){
    //n.glidemotor.set(0);
    return true;}
    else if(n.glidemotor.getReverseLimitSwitch().isPressed() && s<=0){
    //n.glidemotor.set(0);
    return true;}
    else{
    return false;
  }
  }
}
