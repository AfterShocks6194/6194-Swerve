package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        // Control Orientation Chooser
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        switch (orientationChooser.getSelected()) {
            case "Field Oriented":
                m_drivetrainSubsystem.drive(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                m_translationXSupplier.getAsDouble(),
                                m_translationYSupplier.getAsDouble(),
                                m_rotationSupplier.getAsDouble(),
                                m_drivetrainSubsystem.getGyroscopeRotation()));
                break;

            case "Robot Oriented":
                m_drivetrainSubsystem.drive(
                        new ChassisSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble()));
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
