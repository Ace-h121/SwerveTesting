// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//vender library
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//wpi/first imports
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//imports from constant
import frc.robot.Constants.MotorConstants;

public class Drivebase extends SubsystemBase {
  AHRS Gyro;

  //Throttle Modules
  CANSparkMax frontRightThrottle;
  CANSparkMax frontLeftThrottle;
  CANSparkMax backRightThrottle;
  CANSparkMax backLeftThrottle;

  //turning modules  
  CANSparkMax frontRightTurn;
  CANSparkMax frontLeftTurn;
  CANSparkMax backRightTurn;
  CANSparkMax backLeftTurn;

  //motor groups
  MotorControllerGroup ThrottleGroup;
  MotorControllerGroup TurnGroup;

  /** Creates a new Drivebase. */
  public Drivebase() {

    Gyro = new AHRS(I2C.Port.kMXP);

    //defining the throttle motors
    frontRightThrottle  = new CANSparkMax(MotorConstants.FRONT_RIGHT_THROTTLE, MotorType.kBrushless);
    frontLeftThrottle   = new CANSparkMax(MotorConstants.FRONT_LEFT_THROTTLE, MotorType.kBrushless);
    backRightThrottle   = new CANSparkMax(MotorConstants.BACK_RIGHT_THROTTLE, MotorType.kBrushless);
    backLeftThrottle    = new CANSparkMax(MotorConstants.BACK_LEFT_THROTTLE, MotorType.kBrushless);

    //defining the turn motors
    frontRightTurn      = new CANSparkMax(MotorConstants.FRONT_RIGHT_TURN, MotorType.kBrushless);
    frontLeftTurn       = new CANSparkMax(MotorConstants.FRONT_LEFT_TURN, MotorType.kBrushless);
    backRightTurn       = new CANSparkMax(MotorConstants.BACK_RIGHT_TURN, MotorType.kBrushless);
    backLeftTurn        = new CANSparkMax(MotorConstants.BACK_LEFT_TURN, MotorType.kBrushless);

    //grouping throttle and turn motors
    ThrottleGroup       = new MotorControllerGroup(backLeftThrottle, frontLeftThrottle, frontRightThrottle, backRightThrottle);
    TurnGroup           = new MotorControllerGroup(backLeftTurn, frontLeftTurn, frontRightTurn, backRightTurn);
  }

  //setting up the drive method
  public void drive(double speedInput, double turnInput){
    ThrottleGroup.set(speedInput);
    TurnGroup.set(turnInput);
  }

  //gyro methods
  public void resetGyro(){
    Gyro.reset();
  }

  public double getAngle(){
    return Gyro.getAngle();
  }

  public AHRS getGyro(){
    return Gyro;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
