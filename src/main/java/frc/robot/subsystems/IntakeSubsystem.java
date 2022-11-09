/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.Constants;

//6194 Specific Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class IntakeSubsystem extends SubsystemBase {


  public CANSparkMax intake = new CANSparkMax(Constants.INTAKE,MotorType.kBrushless);
  public CANSparkMax conveyor = new CANSparkMax(Constants.CONVEYOR,MotorType.kBrushless);

    // DriveSubsystem is a singleton class as it represents a physical subsystem
    public static IntakeSubsystem currentInstance;

    
    /**
     * Creates a new DriveSubsystem.
     */
    public IntakeSubsystem() {
        CoastMode();
        intake.setInverted(false);
        conveyor.setInverted(true);

        // leftMaster.enableVoltageCompensation(true);
        // rightMaster.enableVoltageCompensation(true);
    }

    private void CoastMode() {
        intake.setIdleMode(IdleMode.kBrake);
        conveyor.setIdleMode(IdleMode.kBrake);
    }


    /**
     * Initialize the current DriveBase instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new IntakeSubsystem();
        }
    }

    public static IntakeSubsystem getInstance() {
        init();
        return currentInstance;
    }

    @Override
    public void periodic() {
        
    }

   
    public void runintake(double speed) {
       intake.set(speed);
    }

    public void runconveyor(double speed){
        conveyor.set(speed);
    }

    public void runboth (double speed) {
        intake.set(speed);
        conveyor.set(speed);
    }
    
}
