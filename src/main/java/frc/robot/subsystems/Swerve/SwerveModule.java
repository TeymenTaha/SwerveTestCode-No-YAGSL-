package frc.robot.subsystems.Swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final SparkAnalogSensor angleEncoder;

    private final SparkClosedLoopController  drivePID;
    private final SparkClosedLoopController  anglePID;

    private double CAO = 0;
    private SwerveModuleState DS = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveCanID, int angleCanID, double chassisAngularOffset){
        driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
        angleMotor = new SparkMax(angleCanID, MotorType.kBrushed);

        driveEncoder = driveMotor.getEncoder();

        angleEncoder = angleMotor.getAnalog();

        drivePID = driveMotor.getClosedLoopController();
        anglePID = angleMotor.getClosedLoopController();
        
        driveMotor.configure(Configs.SwerveXS.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(Configs.SwerveXS.angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        CAO = chassisAngularOffset;
        driveEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(),
        Rotation2d.fromRadians(angleEncoder.getPosition() - CAO));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRadians(angleEncoder.getPosition() - CAO));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(CAO));

    // Thrifty encoder'dan gelen radyan değerine göre optimizasyon
    correctedDesiredState.optimize(Rotation2d.fromRadians(angleEncoder.getPosition()));

    

    drivePID.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    
    // Turning için SparkMax PID'si analog sensörü feedback olarak kullanır
    anglePID.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public double getAngle() {
    return angleEncoder.getPosition();
  }

  
}
