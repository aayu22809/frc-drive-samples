package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.MechConstants;
import frc.robot.systems.MBRFSMv2;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

public class RevShooterUntimed extends Command {
	private CANSparkMax shooterLeftMotor;
	private CANSparkMax shooterRightMotor;
	private MBRFSMv2 mbrFSM;

	/**
	 * Makes a command that shoots the note out.
	 */
	public RevShooterUntimed(MBRFSMv2 mbrFSM) {
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
		mbrFSM.setShooterLeftMotorPower(-MechConstants.SHOOTING_POWER);
		mbrFSM.setShooterRightMotorPower(MechConstants.SHOOTING_POWER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		mbrFSM.setShooterLeftMotorPower(0);
		mbrFSM.setShooterRightMotorPower(0);
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
