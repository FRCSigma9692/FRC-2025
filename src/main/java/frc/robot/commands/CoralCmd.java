package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

public class CoralCmd extends Command {

  ElevatorSub n;
  boolean c;

  public CoralCmd(ElevatorSub n, boolean check){
    this.n = n;
    this.c = check;
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
    //n.passTheCoral();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
