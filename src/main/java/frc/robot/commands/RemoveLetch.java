package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangingSub;

public class RemoveLetch extends Command {

  HangingSub n;

  double speed;


  public RemoveLetch(HangingSub n,double spd) {
    this.n = n;
    speed = spd;
    addRequirements(n);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    n.Servo(speed);    
  
  }

  @Override
  public void end(boolean interrupted) {

    n.Servo(0.5);
  }

  @Override
  public boolean isFinished() {
  
    return false;
  }
}
