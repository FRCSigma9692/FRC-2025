package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TriggerSub;

public class LeftTriggerCmd extends Command {

  TriggerSub n;
  double s;
  Timer timer;

  public LeftTriggerCmd(TriggerSub n, double s) {
    this.n = n;
    this.s = s;
    addRequirements(n);
  }

  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute(){
    n.LeftIntake(s);
  }

  @Override
  public void end(boolean interrupted) {
    //n.passTheCoral();
  }

  @Override
  public boolean isFinished() {
    //if(timer.get() >= 0.5)
    // {n.stop();
    //return true;
    
    return false;
}
    
}
