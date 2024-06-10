package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.MechConstants;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

public class RevShooterUntimed extends Command {
	private CANSparkMax shooterLeftMotor;
	private CANSparkMax shooterRightMotor;

	/**
	 * Makes a command that shoots the note out.
	 */
	public RevShooterUntimed() {
		// Use addRequirements() here to declare subsystem dependencies.
		shooterLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		shooterRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);
		//addRequirements(...);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
		shooterRightMotor.set(MechConstants.SHOOTING_POWER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
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
