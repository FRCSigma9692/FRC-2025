package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TriggerSub;

public class JoyTriggerCmd extends Command {

  TriggerSub n;
  double s;
  Timer timer;

  public JoyTriggerCmd(TriggerSub n, double s) {
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
    n.Intake(s);
  }

  @Override
  public void end(boolean interrupted) {
    //n.passTheCoral();
  }

  @Override
  public boolean isFinished() {
  //   if(s<0 && timer.get() >= 1){
  //   // {n.stop();
  //   return true;
  //   }
  //   else if(s>0 && timer.get() >= 0.7)
  //   {
  //   return true;
  // }
  //   else{
      return false;
    // }
}
    
}
