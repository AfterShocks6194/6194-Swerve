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

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterSubsystem extends SubsystemBase {


  public CANSparkMax shooterright = new CANSparkMax(Constants.RIGHTSHOOTER,MotorType.kBrushless);
  public CANSparkMax shooterleft = new CANSparkMax(Constants.LEFTSHOOTER,MotorType.kBrushless);
  public MotorControllerGroup shooter = new MotorControllerGroup(shooterright, shooterleft);

//   public BangBangController controller = new BangBangController();
//   public RelativeEncoder shootEncoder;


  // DriveSubsystem is a singleton class as it represents a physical subsystem
    public static ShooterSubsystem currentInstance;

    

    public ShooterSubsystem() {
        CoastMode();
        shooterright.setInverted(false);
        shooterleft.setInverted(false);
        shooterright.setOpenLoopRampRate(Constants.RampRate);
        shooterleft.setOpenLoopRampRate(Constants.RampRate);
        // shootEncoder = shooterleft.getEncoder();
        // leftMaster.enableVoltageCompensation(true);
        // rightMaster.enableVoltageCompensation(true);
    }

    private void CoastMode() {
        shooterleft.setIdleMode(IdleMode.kCoast);
        shooterright.setIdleMode(IdleMode.kCoast);
    }


    /**
     * Initialize the current DriveBase instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new ShooterSubsystem();
        }
    }

    public static ShooterSubsystem getInstance() {
        init();
        return currentInstance;
    }

    @Override
    public void periodic() {
        
    }


   public void goboom(double speed) {
       shooter.set(speed);;
   }
    
}
