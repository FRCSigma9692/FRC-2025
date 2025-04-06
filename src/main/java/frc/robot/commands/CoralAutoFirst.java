package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

public class CoralAutoFirst extends Command {

  ElevatorSub n;
  Timer timer;
  
  public CoralAutoFirst(ElevatorSub n){
    this.n = n;
    addRequirements(n);
  }

  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute() {
    n.drop(0.35);
  }

  @Override
  public void end(boolean interrupted) {
    //n.drop(0);
  }

  @Override
  public boolean isFinished() {
    return (!n.CoralIr.get() || timer.get()>=3);
  }
}
