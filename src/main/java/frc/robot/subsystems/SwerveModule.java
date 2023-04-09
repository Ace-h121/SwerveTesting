// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.module.ModuleDescriptor;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */

/*
 Class used to create eaiser to read subsystems so it is not needed to keep remaking the objects
 */
public class SwerveModule {
    //the two motors in every swerve module 
    /*
     * TODO: Switch these to Talon motors instead
     */
    private final CANSparkMax driverMotor;
    private final CANSparkMax turningMotor;

    //Encoders built into the falcon motors
    private final RelativeEncoder driverEncoder;
    private final RelativeEncoder turningEncoder;

    //PID controller used to control turning in auto, also used for setpoint 
    private final PIDController turningPIDControl;

    //Encoder used to see the previous position as well
    private final AnalogInput absouluteEncoder;
    
    private final double absouluteEncoderOffset;

    private boolean absouluteEncoderReversed;

    //Constuctor for init the modules
    public SwerveModule(int turningId, int driveId, boolean driveReversed, boolean turningReversed, int absouluteEncoderID, double absouluteEncoderOffset, boolean absouluteEncoderReversed){
        this.absouluteEncoderOffset = absouluteEncoderOffset;
        this.absouluteEncoderReversed = absouluteEncoderReversed;
        absouluteEncoder = new AnalogInput(absouluteEncoderID);

        driverMotor = new CANSparkMax(driveId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningId, MotorType.kBrushless);

        driverMotor.setInverted(driveReversed);
        turningMotor.setInverted(turningReversed);

        driverEncoder = driverMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        driverEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_ROTATIONS);
        turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_ROTATIONS);
        driverEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_SPEED);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_SPEED);
        
        turningPIDControl = new PIDController(ModuleConstants.kP_TURNING_MOTOR, 0 ,0 );

        turningPIDControl.enableContinuousInput(-Math.PI, Math.PI);
    }

    //methods for auto and path planning

    public double getDrivePosition(){
        return driverEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driverEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        //using the average volatage draw of the encoder dividing it by the total voltage to get the position of thhe wheel based off the encoder
        double angle = absouluteEncoder.getAverageVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle -= absouluteEncoderOffset;
        return angle * (absouluteEncoderReversed ? -1.0 : 1);

    }

    public void resetEncoders() {
        driverEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state) {

        if(Math.abs(state.speedMetersPerSecond) > 0.1){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driverMotor.set(state.speedMetersPerSecond / ModuleConstants.MAX_SPEED);
        turningMotor.set(turningPIDControl.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve " + absouluteEncoder.getChannel() + " state", state.toString());
    }

    public void stop(){
        driverMotor.set(0);
        turningMotor.set(0);
    }
}
