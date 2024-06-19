package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.MechConstants;
import frc.robot.systems.MBRFSMv2;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;

public class IntakeNoteUntimed extends Command {
	private TalonFX intakeMotor;
	private MBRFSMv2 mbrFSM;

	/**
	 * Makes a command that shoots the note out.
	 */
	public IntakeNoteUntimed(MBRFSMv2 mbrFSM) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.mbrFSM = mbrFSM;
		//addRequirements(...);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		mbrFSM.setIntakeMotorPower(MechConstants.INTAKE_POWER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		mbrFSM.setIntakeMotorPower(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
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
