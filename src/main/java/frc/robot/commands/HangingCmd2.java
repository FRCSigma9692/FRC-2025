// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangingSub;

public class HangingCmd2 extends Command {

  HangingSub m;
  double s;

  /** Creates a new MotorCommand. */
  public HangingCmd2(HangingSub m, double speed) {
    this.m = m;
    this.s = speed;
    addRequirements(m);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m.rotateMoter2(s);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m.rotateMoter2(0);
  }

  // Returns true when the command should end.
  @Override
   public boolean isFinished() {
    return false;
  }
}
