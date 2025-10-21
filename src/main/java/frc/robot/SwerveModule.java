package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    // This is the ID number for the module
    public int moduleNumber;
    // This is the offset for the angle sensor
    private Rotation2d angleOffset;

    // Motors for turning and driving
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    // Sensor to measure the angle
    private CANcoder angleEncoder;

    // Helps control the drive motor speed
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    // Open-loop control for the drive motor
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    // Closed-loop control for the drive motor
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    // Controls the angle motor position
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    // Sets up the swerve module
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        // Set up the angle sensor
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        // Set up the angle motor
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute(); // Match the motor position to the sensor

        // Set up the drive motor
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0); // Reset the motor position
    }

    // Moves the module to the desired angle and speed
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        // Adjust the angle to avoid unnecessary spinning
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        // Move the angle motor to the desired angle
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        // Set the speed of the drive motor
        setSpeed(desiredState, isOpenLoop);
    }

    // Sets the speed of the drive motor
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            // Open-loop: just set the motor power
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            // Closed-loop: use velocity and feedforward for precise control
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    // Gets the current angle from the sensor
    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    // Resets the angle motor to match the sensor's absolute position
    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    // Gets the current speed and angle of the module
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    // Gets the distance traveled and current angle of the module
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }
}