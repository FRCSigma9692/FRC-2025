// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

public class ElevatorCmd extends Command {

  ElevatorSub m;
  double s;

  /** Creates a new MotorCommand. */
  public ElevatorCmd(ElevatorSub m, double pos) {
    this.m = m;
    this.s = pos;
    addRequirements(m);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m.rotateMoter(s);
    //m.powerBase(s);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      //m.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m.LeftElevator.getPosition().getValueAsDouble())>=(s-4))
    return true;

    return false;
    
  }
}
