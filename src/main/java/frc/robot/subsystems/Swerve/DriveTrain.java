
package frc.robot.subsystems.Swerve;


import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveContants;

public class DriveTrain extends SubsystemBase{
    
    private final SwerveModule frontLeft = new SwerveModule(DriveContants.flDriveCanID, DriveContants.flAngleCanID, DriveContants.flChassisAngularOffset);
    private final SwerveModule frontRight = new SwerveModule(DriveContants.frDriveCanID, DriveContants.frAngleCanID, DriveContants.frChassisAngularOffset);
    private final SwerveModule backLeft = new SwerveModule(DriveContants.blDriveCanID, DriveContants.blAngleCanID, DriveContants.blChassisAngularOffset);
    private final SwerveModule backRight = new SwerveModule(DriveContants.brDriveCanID, DriveContants.brAngleCanID, DriveContants.brChassisAngularOffset);

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveDriveOdometry odometry;

    public DriveTrain(){
        zeroHeading();

        odometry = new SwerveDriveOdometry(
        DriveContants.kDriveKinematics,
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    }

    @Override
    public void periodic() {
        // Odometry güncelleme (Robotun sahadaki yerini hesaplar)
        odometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });

        // SmartDashboard üzerinden tekerlek açılarını ve gyro bilgisini takip et
        SmartDashboard.putNumber("FL Angle Deg", Math.toDegrees(frontLeft.getAngle()));
        SmartDashboard.putNumber("FR Angle Deg", Math.toDegrees(frontRight.getAngle()));
        SmartDashboard.putNumber("RL Angle Deg", Math.toDegrees(backLeft.getAngle()));
        SmartDashboard.putNumber("RR Angle Deg", Math.toDegrees(backRight.getAngle()));
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * DriveContants.maxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveContants.maxSpeedMetersPerSecond;
        double rotDelivered = rotation * DriveContants.maxAngularSpeed;

        var swerveModuleStates = DriveContants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        
        setModuleStates(swerveModuleStates);
    }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Tekerlek hızlarını maksimum hıza oranla (Desaturate)
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveContants.maxSpeedMetersPerSecond);
    
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

    public Rotation2d getRotation2d() {
        // NavX açısını WPILib standartlarına (Saat yönü tersi pozitif) çeviriyoruz
        return Rotation2d.fromDegrees(DriveContants.gyroReversed ? navx.getAngle() : -navx.getAngle());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(), frontRight.getPosition(),
                backLeft.getPosition(), backRight.getPosition()
            }, pose);
    }

    public void zeroHeading() {
        navx.reset();
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
    
      /** Tekerlekleri X şeklinde kilitler (Defans modu) */
      public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      }
}

