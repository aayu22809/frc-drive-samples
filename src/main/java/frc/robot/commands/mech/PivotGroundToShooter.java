// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
// import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

// Robot Imports
import frc.robot.HardwareMap;
import frc.robot.MechConstants;
import frc.robot.systems.MBRFSMv2;


public class PivotGroundToShooter extends Command {
	private TalonFX intakeMotor;
	private TalonFX pivotMotor;
	private Encoder throughBore;
	private Timer timer;
	private MBRFSMv2 mbrFSM;

	/**
	 * IntakeGroundToShooter command.
	 */
	public PivotGroundToShooter(MBRFSMv2 mbrFSM) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.mbrFSM = mbrFSM;
		timer = new Timer();
		//addRequirements(m_intakeWheels);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		mbrFSM.setIntakeMotorPower(MechConstants.AUTO_HOLDING_POWER);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		mbrFSM.pidPivotAuto(MechConstants.SHOOTER_ENCODER_ROTATIONS);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return mbrFSM.pidPivotCompleted(MechConstants.SHOOTER_ENCODER_ROTATIONS);
	}

	private double pidAuto(double currentEncoderPID, double targetEncoder) {
		double correction = MechConstants.PID_CONSTANT_PIVOT_P_AUTO
			* (targetEncoder - currentEncoderPID);

		return Math.min(MechConstants.MAX_TURN_SPEED_AUTO,
			Math.max(MechConstants.MIN_TURN_SPEED_AUTO, correction));
	}

	private boolean inRange(double a, double b) {
		return Math.abs(a - b) < MechConstants.INRANGE_VALUE; //EXPERIMENTAL
	}
}
