package frc.robot.commands.align;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.MechConstants;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.DriveFSMSystem;

public class AprilTagAlign extends Command {
	private int atID;
	private DriveFSMSystem driveSubsystem;
	private Timer timer;
	private double timeAlign;

	/**
	 * Makes a command that aligns robot to pose using DriveFSMSystem.
	 * @param id id of AT to align with
	 * @param drive_subsystem Drive Subsystem with alignment functions
	 * @param timeAlign the amount of time that the robot aligns with the tag
	 */
	public AprilTagAlign(int id, DriveFSMSystem driveSubsystem, double timeAlign) {
		atID = id;
		this.driveSubsystem = driveSubsystem;
		this.timeAlign = timeAlign;
		timer = new Timer();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (timer.get() == 0) {
			timer.start();
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		driveSubsystem.alignToSpeaker(atID);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		driveSubsystem.autoResetAlignment();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() > timeAlign || driveSubsystem.speakerAligned();
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
