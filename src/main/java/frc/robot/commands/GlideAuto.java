package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GlideSub;

public class GlideAuto extends Command {

  GlideSub n;
  boolean flag;

  public GlideAuto(GlideSub n) {
    this.n = n;
    addRequirements(n);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //n.moveBit();
    //n.drop(s);
  }

  @Override
  public void end(boolean interrupted) {
    //n.passTheCoral(0);
    //n.drop(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
