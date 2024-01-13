package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX rotationMotor;
    private CANCoder rotationEncoder;
    private SwerveModuleState currentState;
    private SwerveModuleState desiredState;

    PIDController drivePIDController;
    PIDController rotationPIDController;

    public SwerveModule(
            int driveMotorPort,
            int rotationMotorPort,
            int encoderPort) {

        driveMotor = new TalonFX(driveMotorPort);
        rotationMotor = new TalonFX(rotationMotorPort);
        rotationEncoder = new CANCoder(encoderPort);
        currentState = new SwerveModuleState();

        driveMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.setNeutralMode(NeutralMode.Brake);

        drivePIDController = new PIDController(0.001, 0, 0);
        rotationPIDController = new PIDController(0.001, 0, 0);
        rotationPIDController.disableContinuousInput();
    }

    public SwerveModuleState getState() {
        return currentState;
    }

    public void stopMotors() {
        driveMotor.set(TalonFXControlMode.PercentOutput, 0);
        rotationMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getDegrees() {
        return rotationEncoder.getAbsolutePosition();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                getDegrees());
    }

    public void setDesiredState(SwerveModuleState newState) {
        // desiredState = newState;
        desiredState = new SwerveModuleState(0.3, new Rotation2d((30)));
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.1) {
            stopMotors();
            return;
        }

        Rotation2d currentRotation = getAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        // Find the difference between our current rotational position + our new
        // rotational position
        Rotation2d rotationDelta = state.angle.minus(currentRotation);
        double desiredRotation = currentRotation.getDegrees() + rotationDelta.getDegrees();

        rotationPIDController.setTolerance(5);

        double rotationOutput = rotationPIDController.calculate(
                currentRotation.getDegrees(),
                desiredRotation);

        
        rotationMotor.set(TalonFXControlMode.PercentOutput,
                MathUtil.clamp(
                        (rotationOutput),
                        -1.0,
                        1.0));

        driveMotor.set(TalonFXControlMode.PercentOutput, state.speedMetersPerSecond / Units.feetToMeters(15.2));
    }
}

public class SwerveSubsystem extends SubsystemBase {

    SwerveModule frontLeftModule = new SwerveModule(18, 17, 2);
    SwerveModule frontRightModule = new SwerveModule(10, 16, 4);
    SwerveModule rearLeftModule = new SwerveModule(11, 14, 1);
    SwerveModule rearRightModule = new SwerveModule(13, 12, 3);

    double chassisWidth = 0.65; // meters
    double chassisLength = 0.65; // meters

    Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
    Translation2d rearLeftLocation = new Translation2d(-chassisLength / 2, chassisWidth / 2);
    Translation2d rearRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth / 2);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            rearLeftLocation,
            rearRightLocation);

    CommandXboxController controller;

    public SwerveSubsystem(CommandXboxController io) {
        System.out.println("SwerveSubsystem constructor");
        controller = io;
    }

    public void setChassisSpeed(ChassisSpeeds desired) {
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desired);

        frontLeftModule.setDesiredState(newStates[0]);
        frontRightModule.setDesiredState(newStates[1]);
        rearLeftModule.setDesiredState(newStates[2]);
        rearRightModule.setDesiredState(newStates[3]);
    }

    @Override
    public void periodic() {

        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
                controller.getLeftY(),

                controller.getLeftX(),

                controller.getRightX());

        setChassisSpeed(newDesiredSpeeds);

        // FL,FR,RL,RR
        double loggingState[] = {
                frontLeftModule.getState().angle.getDegrees(),
                frontLeftModule.getState().speedMetersPerSecond,
                frontRightModule.getState().angle.getDegrees(),
                frontRightModule.getState().speedMetersPerSecond,
                rearLeftModule.getState().angle.getDegrees(),
                rearLeftModule.getState().speedMetersPerSecond,
                rearRightModule.getState().angle.getDegrees(),
                rearRightModule.getState().speedMetersPerSecond,
        };

        SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
    }
}
