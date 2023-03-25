// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivebase;

public class Drive extends CommandBase {

  Drivebase drivebase;

  XboxController driver;

  double throttle;
  double turn;

  boolean brake;

  public Drive(Drivebase drivebase) {

    drivebase = this.drivebase;

    driver = RobotContainer.getDriverController();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    throttle = driver.getLeftY();
    turn = driver.getRightX();
    brake = driver.getRightBumper();

    if(brake){
      drivebase.drive(0,0);
    }

    if(throttle > ControllerConstants.AXIS_THRESHOLD || throttle < ControllerConstants.NEGITIVE_AXIS_THRESHOLD){
      if(turn > ControllerConstants.AXIS_THRESHOLD || turn < ControllerConstants.NEGITIVE_AXIS_THRESHOLD){
        drivebase.drive(throttle, turn);
      }

      else if(turn > ControllerConstants.AXIS_THRESHOLD || turn < ControllerConstants.NEGITIVE_AXIS_THRESHOLD){
        drivebase.drive(0, turn);
      }
      
      else{
        drivebase.drive(throttle, 0);
      }
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
