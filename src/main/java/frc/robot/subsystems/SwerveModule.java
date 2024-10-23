package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor; 
    private final CANSparkMax turningMotor;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder; 
    private final CANcoder absoluteEncoder;

    private final PIDController turningPIDController;
    private final PIDController velocityPIDController;

    private SwerveModuleState desiredState;
    private double offset;

    public SwerveModule(int driveMotorID, int turningMotorID, int CANCoderID, double offset) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        absoluteEncoder = new CANcoder(CANCoderID);

        this.offset = offset; // offset is in degrees

        turningEncoder.setPositionConversionFactor(1.0/SwerveConstants.GEAR_RATIO);
        driveEncoder.setPositionConversionFactor(1.0/SwerveConstants.DRIVE_GEAR_RATIO);
        driveEncoder.setVelocityConversionFactor(1.0/SwerveConstants.DRIVE_GEAR_RATIO);

        velocityPIDController = new PIDController(0.1, 0, 0.01);


        turningPIDController = new PIDController(0.25, 0, 0.01);
		turningPIDController.enableContinuousInput(0, 2 * Math.PI);

        desiredState = new SwerveModuleState();
    }


    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getEncoderAngle()));
    }

    public void setEncoder(){
        turningEncoder.setPosition((1.0 - (absoluteEncoder.getAbsolutePosition().getValueAsDouble() + (offset/360))) % 1.0);
    }

    public void periodic() {
        // Set the drive motor speed
        driveMotor.set(velocityPIDController.calculate(driveEncoder.getVelocity(), getDiesiredRPMs()));

        // Set the turning motor position
		double angle = getEncoderAngle();
        double desiredAngle = desiredState.angle.getRadians();
        turningMotor.set(turningPIDController.calculate(angle, desiredAngle));
    }

    double getEncoderAngle(){ // in radians
        return (turningEncoder.getPosition() % 1.0) * 2 * Math.PI;
    }

    double getDiesiredRPMs(){
        return (desiredState.speedMetersPerSecond / (Math.PI * SwerveConstants.WHEEL_DIAMETER)) * 60.0;
    }


    public SwerveModulePosition getPosition() {
        double position = driveEncoder.getPosition() * SwerveConstants.WHEEL_CIRCUMFERENCE;
        Rotation2d angle = Rotation2d.fromRadians(getEncoderAngle());
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            Rotation2d.fromRadians(getEncoderAngle())
        );
    }

}
