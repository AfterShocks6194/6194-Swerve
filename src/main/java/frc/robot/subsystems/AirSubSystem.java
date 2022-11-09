/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants;

//6194 Specific Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AirSubSystem extends SubsystemBase {

    public final PneumaticHub m_ph = new PneumaticHub(Constants.AIR);

    // pnuematic definitions below
    private static int climberUp = 0;
    private static int climberDown = 1;
    DoubleSolenoid Climber = m_ph.makeDoubleSolenoid(climberUp, climberDown);
    private static int intakeOut = 2;
    private static int intakeIn = 3;
    DoubleSolenoid Intake = m_ph.makeDoubleSolenoid(intakeIn, intakeOut);
    private static int elevatorUp = 5;
    private static int elevatorDown = 4;
    DoubleSolenoid Elevator = m_ph.makeDoubleSolenoid(elevatorUp, elevatorDown);
    


    public void ClimberUp() {
        Climber.set(DoubleSolenoid.Value.kForward);
    }

    public void ClimberDown() {
        Climber.set(DoubleSolenoid.Value.kReverse);
    }

    public void SolenoidOff() {
        Climber.set(DoubleSolenoid.Value.kOff);
    }

    public void toggleIntake(){
        Intake.toggle();
    }

    public void toggleClimber(){
        Climber.toggle();
    }

    public void toggleElevator(){
        Elevator.toggle();
    }

    public void elevatorUp(){
        Elevator.set(Value.kForward);
    }

    public void elevatorDown(){
        Elevator.set(Value.kReverse);
    }


    // // AirSubsystem is a singleton class as it represents a physical subsystem
    public static AirSubSystem currentInstance;

    /**
     * Creates a new AirSubsystem.
     */
    public AirSubSystem() {
        SmartDashboard.setDefaultBoolean("Enable Compressor Analog", true);
        SmartDashboard.setDefaultBoolean("Disable Compressor", false);
        // Add number inputs for minimum and maximum pressure
        SmartDashboard.setDefaultNumber("Minimum Pressure (PSI)", 100.0);
        SmartDashboard.setDefaultNumber("Maximum Pressure (PSI)", 120.0);

    }

    /**
     * Initialize the current DriveBase instance
     */
    public static void init() {
        if (currentInstance == null) {
            currentInstance = new AirSubSystem();
        }
    }

    public static AirSubSystem getInstance() {
        init();
        return currentInstance;
    }

    @Override
    public void periodic() {
        /*** Get pressure from analog channel 0 and display on Shuffleboard. */
        SmartDashboard.putNumber("Pressure", m_ph.getPressure(0));

        /*** Get compressor running status and display on Shuffleboard. */
        SmartDashboard.putBoolean("Compressor Running", m_ph.getCompressor());

        // Enable Compressor Analog button
        if (SmartDashboard.getBoolean("Enable Compressor Analog", false)) {
            SmartDashboard.putBoolean("Enable Compressor Analog", false);

            // Get values from Shuffleboard
            double minPressure = SmartDashboard.getNumber("Minimum Pressure (PSI)", 0);
            double maxPressure = SmartDashboard.getNumber("Maximum Pressure (PSI)", 0);

            /**
             * Enable the compressor with analog pressure sensor control.
             *
             * This uses hysteresis between a minimum and maximum pressure value,
             * the compressor will run when the sensor reads below the minimum pressure
             * value, and the compressor will shut off once it reaches the maximum.
             *
             *
             */
            m_ph.enableCompressorAnalog(minPressure, maxPressure);
        }

        // Disable Compressor button
        if (SmartDashboard.getBoolean("Disable Compressor", false)) {
            SmartDashboard.putBoolean("Disable Compressor", false);

            /**
             * Disable the compressor.
             */
            m_ph.disableCompressor();
        }

    }

}
