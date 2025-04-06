package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TriggerSub;

public class TriggerCmd extends Command {

  TriggerSub n;
  double s;
  Timer timer;

  public TriggerCmd(TriggerSub n, double s) {
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
    if(s>0)
    n.TriggerPID(s);
    else
    n.Intake(s);

    
  }

  @Override
  public void end(boolean interrupted) {
    //n.passTheCoral();
    n.stop();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    if((n.triggermotor.getForwardLimitSwitch().isPressed() && s>0 || n.enc.getPosition()>=26000 && s>0) || (timer.get()>0.7 && s>0)){//s<0 && timer.get() >= 1){
    // {n.stop();
    return true;
    }
    else if(n.triggermotor.getReverseLimitSwitch().isPressed() && s<0){//s>0 && timer.get() >= 0.7)
        
    return true;
  }
    else{
      return false;
    }
}
    
}
