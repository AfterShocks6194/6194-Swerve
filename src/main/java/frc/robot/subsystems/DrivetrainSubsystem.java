// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = Constants.MAX_VOLTAGE;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.MAX_VELOCITY_METERS_PER_SECOND;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        private final SwerveDriveKinematics m_kinematics = Constants.m_kinematics;

        private final AHRS m_navx = new AHRS(Port.kMXP, (byte) 200); // NavX connected over MXP port

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        // Odometry class for tracking robot pose
        public SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());

        public final Field2d m_field = new Field2d();

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                // Setup motor configuration
                m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                // This is the ID of the drive motor
                                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                // This is the ID of the steer motor
                                FRONT_LEFT_MODULE_STEER_MOTOR,
                                // This is the ID of the steer encoder
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                FRONT_LEFT_MODULE_STEER_OFFSET);

                // We will do the same for the other modules
                m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L1,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_navx.zeroYaw();
                m_navx.resetDisplacement();
        }

        public Rotation2d getGyroscopeRotation() {

                if (m_navx.isMagnetometerCalibrated()) {
                        // We will only get valid fused headings if the magnetometer is calibrated
                        return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                }
                //
                // // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        }

        public double heading(){
                if (m_navx.isMagnetometerCalibrated()) {
                        // We will only get valid fused headings if the magnetometer is calibrated
                        return (m_navx.getFusedHeading());
                }
                //
                // // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return (360.0 - m_navx.getYaw());
        }

        public void navx_odometry(){
                SmartDashboard.putNumber("NavX X Coordinates", m_navx.getDisplacementX());
                SmartDashboard.putNumber("NavX Y Coordinates", m_navx.getDisplacementY());
                
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                setModuleStates(states);
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Robot Heading", heading());
                SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
                navx_odometry();
                SmartDashboard.updateValues();
                SmartDashboard.putData("Field", m_field);
                m_field.setRobotPose(m_odometry.getPoseMeters());
                

                //This breaks things? 
        //         m_odometry.update(getGyroscopeRotation(),
        //         new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
        //         new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle())),
        //         new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle())),
        //         new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle()))
        // );
        }

        //Returns the currently-estimated pose of the robot. @return The pose.
        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        /**
         * Resets the odometry to the specified pose.
         * @param pose The pose to which to set the odometry.
         */
        public void resetOdometry(Pose2d pose) {
                m_odometry.resetPosition(pose, getGyroscopeRotation());
        }

        public void stopModules() {
                m_frontLeftModule.set(0, 0);
                m_frontRightModule.set(0, 0);
                m_backLeftModule.set(0, 0);
                m_backRightModule.set(0, 0);
        }

        public void setModuleStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kPhysicalMaxSpeedMetersPerSecond);
                m_frontLeftModule.set(
                                desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[0].angle.getRadians());
                m_frontRightModule.set(
                                desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[1].angle.getRadians());
                m_backLeftModule.set(
                                desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[2].angle.getRadians());
                m_backRightModule.set(
                                desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                desiredStates[3].angle.getRadians());
                StatestoOdometry(desiredStates);

                
        }

        public void StatestoOdometry(SwerveModuleState[] odo_ModuleStates){
                m_odometry.update(getGyroscopeRotation(), odo_ModuleStates[0], odo_ModuleStates[1], odo_ModuleStates[2], odo_ModuleStates[3]);
        }

        public void ResetAbsolute(){
        }

}
