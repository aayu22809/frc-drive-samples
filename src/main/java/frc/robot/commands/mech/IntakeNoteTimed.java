package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.MechConstants;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;

public class IntakeNoteTimed extends Command {
	private TalonFX intakeMotor;
	private Timer timer;
	private float timeIntaking;

	/**
	 * Makes a command that shoots the note out.
	 * @param timeIntaking time the intake outtakes the note
	 */
	public IntakeNoteTimed(float timeIntaking) {
		// Use addRequirements() here to declare subsystem dependencies.
		intakeMotor = new TalonFX(HardwareMap.DEVICE_ID_INTAKE_MOTOR);
		intakeMotor.setNeutralMode(NeutralModeValue.Brake);

		timer = new Timer();
		this.timeIntaking = timeIntaking;

		//addRequirements(...);

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
		intakeMotor.set(MechConstants.INTAKE_POWER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intakeMotor.set(0);
		timer.stop();
		timer.reset();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() > timeIntaking;
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
