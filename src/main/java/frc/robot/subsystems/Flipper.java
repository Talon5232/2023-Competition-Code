// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** Add your docs here. */
public class Flipper {  
    private int keepFlipperOn = 0; 
    private final TalonSRX FlipperMotor;
    public Flipper(){
        FlipperMotor = new TalonSRX(0);
            //NEED DEVICED ID
    }   
    


    public void Flipperin(){
        FlipperMotor.set(TalonSRXControlMode.PercentOutput, .6);
        
    }
    public void Flipperout(){
        FlipperMotor.set(TalonSRXControlMode.PercentOutput, -.6);
        keepFlipperOn = 2;
        keepFlipperOn = 2;
       
    }
    public void FlipperStop(){
        FlipperMotor.set(TalonSRXControlMode.PercentOutput, 0);
        keepFlipperOn = 2;
        keepFlipperOn = 2;
}}
