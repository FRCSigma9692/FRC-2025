package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.TriggerSub;

public class TriggerCmdSensor extends Command {

  TriggerSub n;
  ElevatorSub e;
  double s;
  Timer timer;

  public TriggerCmdSensor(TriggerSub n, double s, ElevatorSub e) {
    this.n = n;
    this.s = s;
    this.e = e;
    addRequirements(n);
  }

  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute(){
    if(!e.CoralIr.get()){
    if(s>0)
    n.TriggerPID(s);
    else
    n.Intake(s);}

    
  }

  @Override
  public void end(boolean interrupted) {
    //n.passTheCoral();
    n.stop();
  }

  @Override
  public boolean isFinished() {
    if(((n.triggermotor.getForwardLimitSwitch().isPressed() || n.enc.getPosition()>=26000) && s>0) || e.CoralIr.get()){//s<0 && timer.get() >= 1){
    // {n.stop();
    return true;
    }
    else if((n.triggermotor.getReverseLimitSwitch().isPressed() && s<0) || e.CoralIr.get()){//s>0 && timer.get() >= 0.7)
        
    return true;
  }
    else{
      return false;
    }
}
    
}
