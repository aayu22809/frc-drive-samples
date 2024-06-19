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

public class OuttakeNote extends Command {
	private TalonFX intakeMotor;
	private Encoder throughBore;
	private Timer timer;
	private double timeOuttaking;
	private MBRFSMv2 mbrFSM;

	/**
	 * Makes a command that shoots the note out.
	 * @param timeOuttaking time the intake outtakes the note
	 */
	public OuttakeNote(double timeOuttaking, MBRFSMv2 mbrFSM) {
		// Use addRequirements() here to declare subsystem dependencies.
		this.mbrFSM = mbrFSM;

		timer = new Timer();
		this.timeOuttaking = timeOuttaking;

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
		mbrFSM.setIntakeMotorPower(MechConstants.OUTTAKE_POWER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		mbrFSM.setIntakeMotorPower(0);
		timer.stop();
		timer.reset();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() > timeOuttaking;
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
