package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

public class CoralAutoNext extends Command {

  ElevatorSub n;
  
  public CoralAutoNext(ElevatorSub n){
    this.n = n;
    addRequirements(n);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    n.passTheCoral();
  }

  @Override
  public void end(boolean interrupted) {
    n.drop(0);
  }

  @Override
  public boolean isFinished() {
    return n.CoralIr.get();
  }
}
