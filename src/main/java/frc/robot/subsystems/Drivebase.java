// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//vender library
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//wpi/first imports
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PortConstants;


public class Drivebase extends SubsystemBase {

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;
  
  AHRS Gyro;

  /** Creates a new Drivebase. */
  public Drivebase() {

    frontLeft = new SwerveModule(
      PortConstants.FRONT_LEFT_TURN,
      PortConstants.FRONT_LEFT_THROTTLE,
      true,
      true,
      PortConstants.FRONT_LEFT_ENCODER_PORT,
      ModuleConstants.FRONT_LEFT_ENCODER_OFFSET,
      ModuleConstants.FRONT_LEFT_ENCODER_REVERSED
    );

    backLeft = new SwerveModule(
      PortConstants.BACK_LEFT_TURN,
      PortConstants.BACK_LEFT_THROTTLE,
      true,
      true,
      PortConstants.BACK_LEFT_ENCODER_PORT,
      ModuleConstants.BACK_LEFT_ENCODER_OFFSET,
      ModuleConstants.BACK_LEFT_ENCODER_REVERSED
    );

    backRight = new SwerveModule(
      PortConstants.BACK_RIGHT_TURN,
      PortConstants.BACK_RIGHT_THROTTLE,
      false,
      true,
      PortConstants.BACK_RIGHT_ENCODER_PORT,
      ModuleConstants.BACK_RIGHT_ENCODER_OFFSET,
      ModuleConstants.BACK_RIGHT_ENCODER_REVERSED
    );

    frontRight = new SwerveModule(
      PortConstants.FRONT_RIGHT_TURN,
      PortConstants.FRONT_RIGHT_THROTTLE,
      false,
      true,
      PortConstants.FRONT_RIGHT_ENCODER_PORT,
      ModuleConstants.FRONT_RIGHT_ENCODER_OFFSET,
      ModuleConstants.FRONT_RIGHT_ENCODER_REVERSED
    );

    Gyro = new AHRS(I2C.Port.kMXP);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetGyro();
      } 
      catch(Exception e) {
      }
    }).start();
  }
  //setting up the drive method


  //gyro methods
  public void resetGyro(){
    Gyro.reset();
  }

  public double getAngle(){
    return Gyro.getAngle();
  }

  public double getHeading(){
    return Math.IEEEremainder(getAngle(), 360);
  }

  public AHRS getGyro(){
    return Gyro;
  }

  public Rotation2d geRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.MAX_SPEED);
    frontLeft.setState(desiredStates[0]);
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
