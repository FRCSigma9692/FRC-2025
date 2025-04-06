package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

public class CoralDropCmd extends Command {

  ElevatorSub n;
  double time;
  double speed;
  Timer timer;

  public CoralDropCmd(ElevatorSub n, double time, double spd) {
    this.n = n;
    this.time = time;
    speed = spd;
    addRequirements(n);

  }

  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute() {
    n.drop(speed);
    
   SmartDashboard.putNumber("TIMER", timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    //n.passTheCoral();
    n.drop(0);
  }

  @Override
  public boolean isFinished() {
    if(timer.get() >= time)
    return true;
    return false;
  }
}
