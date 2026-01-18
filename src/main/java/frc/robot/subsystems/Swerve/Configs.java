package frc.robot.subsystems.Swerve;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ModuleConstants;

public class Configs {
    public static class SwerveXS {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig angleConfig = new SparkMaxConfig();

        static {
            // --- Drive Ayarları (NEO) ---
            double driveFactor = ModuleConstants.wheelDiameterMeters * Math.PI
                                   / ModuleConstants.drivingMotorReduction;
            double driveVelocityFeedForward = 1 / ModuleConstants.driveWheelFreeSpeedRps;

            driveConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.drivingMotorCurrentLimit);

            driveConfig.encoder
                    .positionConversionFactor(driveFactor) // metre
                    .velocityConversionFactor(driveFactor / 60.0); // metre/saniye

            driveConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .velocityFF(driveVelocityFeedForward)
                    .outputRange(-1, 1);


            double angleFactor = 2 * Math.PI;

            angleConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.turningMotorCurrentLimit);

            angleConfig.encoder
                    .positionConversionFactor(angleFactor)
                    .velocityConversionFactor(angleFactor / 60.0);

            angleConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAnalogSensor)
                    .pid(1.0, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, angleFactor);
        }
    }
}
