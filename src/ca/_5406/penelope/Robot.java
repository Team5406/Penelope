package ca._5406.penelope;

import ca._5406.util.CheesyDriveHelper;
import ca._5406.util.Looper;
import ca._5406.util.Looper.Loopable;
import ca._5406.util.XboxController;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static PowerDistributionPanel pdp;
	public static Compressor comp;

	private XboxController driverGamepad = new XboxController(0);
	private Drive drive = Drive.getInstance();
	private CheesyDriveHelper cheesyDrive = CheesyDriveHelper.getInstance();

	private Looper controlModeLooper = new Looper(this::update, 0.05);

	private SendableChooser<ControlMode> controlModeChooser;
	private SendableChooser<ShiftMode> shiftModeChooser;
	private SendableChooser<NeutralMode> neutralModeChooser;

	private ControlMode selectedControlMode = ControlMode.ARCADE;
	private ShiftMode selectedShiftMode = ShiftMode.HOLD_B_TO_BOOST;
	private NeutralMode selectedNeutralMode = NeutralMode.Coast;
	private boolean squareThrottle = false;
	private boolean squareTurning = false;
	private boolean currentLimiting = false;
	private boolean switchPrimaryStick = false;

	@Override
	public void robotInit() {
		pdp = new PowerDistributionPanel(15);
		comp = new Compressor(0);
		controlModeLooper.start();

		controlModeChooser = new SendableChooser<>();
		controlModeChooser.addDefault(ControlMode.ARCADE.name(), ControlMode.ARCADE);
		controlModeChooser.addObject(ControlMode.SPLIT_ARCADE.name(), ControlMode.SPLIT_ARCADE);
		controlModeChooser.addObject(ControlMode.TANK.name(), ControlMode.TANK);
		controlModeChooser.addObject(ControlMode.CHEESY.name(), ControlMode.CHEESY);
		controlModeChooser.addObject(ControlMode.SINGLE_STICK_CHEESY.name(), ControlMode.SINGLE_STICK_CHEESY);
		controlModeChooser.addObject(ControlMode.CHEESY_CAR.name(), ControlMode.CHEESY_CAR);
		controlModeChooser.addObject(ControlMode.CAR.name(), ControlMode.CAR);

		shiftModeChooser = new SendableChooser<>();
		shiftModeChooser.addDefault(ShiftMode.HOLD_B_TO_BOOST.name(), ShiftMode.HOLD_B_TO_BOOST);
		shiftModeChooser.addObject(ShiftMode.HOLD_B_TO_PUSH.name(), ShiftMode.HOLD_B_TO_PUSH);
		shiftModeChooser.addObject(ShiftMode.HOLD_RB_TO_BOOST.name(), ShiftMode.HOLD_RB_TO_BOOST);
		shiftModeChooser.addObject(ShiftMode.HOLD_RB_TO_PUSH.name(), ShiftMode.HOLD_RB_TO_PUSH);
		shiftModeChooser.addObject(ShiftMode.TWO_BUTTON_TOGGLE.name(), ShiftMode.TWO_BUTTON_TOGGLE);

		neutralModeChooser = new SendableChooser<>();
		neutralModeChooser.addDefault(NeutralMode.Coast.name(), NeutralMode.Coast);
		neutralModeChooser.addObject(NeutralMode.Brake.name(), NeutralMode.Brake);
		
		SmartDashboard.putBoolean("Switch Control Sides", false);
		SmartDashboard.putBoolean("Square Throttle", false);
		SmartDashboard.putBoolean("Square Turning", false);
		SmartDashboard.putData("Control Mode", controlModeChooser);
		SmartDashboard.putData("Shift Mode", shiftModeChooser);
		SmartDashboard.putData("Neutral Mode", neutralModeChooser);
	}
	
	public void teleopInit(){
		update();
		drive.brownoutMonitor.updateBatteryVoltage();
	}

	@Override
	public void teleopPeriodic() {
		driverGamepad.updateButtons();

		double leftY = driverGamepad.getLeftY();
		double leftX = driverGamepad.getLeftX();
		double rightY = driverGamepad.getRightY();
		double rightX = driverGamepad.getRightX();
		double leftTrigger = driverGamepad.getLeftTrigger();
		double rightTrigger = driverGamepad.getRightTrigger();
		boolean xButtonHeld = driverGamepad.getButtonHeld(XboxController.X_BUTTON);

		if (squareThrottle) {
			leftY *= Math.abs(leftY);
			rightY *= Math.abs(rightY);
			leftTrigger *= Math.abs(leftTrigger);
			rightTrigger *= Math.abs(rightTrigger);
		}
		if (squareTurning) {
			leftX *= Math.abs(leftX);
			rightX *= Math.abs(rightX);
		}

		switch (selectedControlMode) {
		default:
		case ARCADE:
			if (switchPrimaryStick) {
				drive.doArcadeDrive(rightY, rightX);
			} else {
				drive.doArcadeDrive(leftY, leftX);
			}
			break;

		case SPLIT_ARCADE:
			if (switchPrimaryStick) {
				drive.doArcadeDrive(rightY, leftX);
			} else {
				drive.doArcadeDrive(leftY, rightX);
			}
			break;

		case TANK:
			drive.driveLeftRight(leftY, rightY);
			break;

		case CHEESY:
			if (switchPrimaryStick) {
				cheesyDrive.updateCheesyDriveValues(rightY, leftX, xButtonHeld, drive.isHighGear());
			} else {
				cheesyDrive.updateCheesyDriveValues(leftY, rightX, xButtonHeld, drive.isHighGear());
			}
			drive.driveLeftRight(cheesyDrive.getLeftValue(), cheesyDrive.getRightValue());
			break;

		case SINGLE_STICK_CHEESY:
			if (switchPrimaryStick) {
				cheesyDrive.updateCheesyDriveValues(rightY, rightX, xButtonHeld, drive.isHighGear());
			} else {
				cheesyDrive.updateCheesyDriveValues(leftY, leftX, xButtonHeld, drive.isHighGear());
			}
			drive.driveLeftRight(cheesyDrive.getLeftValue(), cheesyDrive.getRightValue());
			break;

		case CHEESY_CAR:
			if (switchPrimaryStick) {
				cheesyDrive.updateCheesyDriveValues(leftTrigger - rightTrigger, rightX, xButtonHeld,
						drive.isHighGear());
			} else {
				cheesyDrive.updateCheesyDriveValues(leftTrigger - rightTrigger, leftX, xButtonHeld, drive.isHighGear());
			}
			drive.driveLeftRight(cheesyDrive.getLeftValue(), cheesyDrive.getRightValue());
			break;

		case CAR:
			if (switchPrimaryStick) {
				drive.doArcadeDrive(rightTrigger - leftTrigger, rightX);
			} else {
				drive.doArcadeDrive(rightTrigger - leftTrigger, leftX);
			}
			break;
		}

		switch (selectedShiftMode) {
		default:
		case HOLD_B_TO_BOOST:
			drive.setGear(driverGamepad.getButtonHeld(XboxController.B_BUTTON) ? Drive.Gear.HIGH : Drive.Gear.LOW);
			break;
		case HOLD_B_TO_PUSH:
			drive.setGear(driverGamepad.getButtonHeld(XboxController.B_BUTTON) ? Drive.Gear.LOW : Drive.Gear.HIGH);
			break;
		case HOLD_RB_TO_BOOST:
			drive.setGear(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) ? Drive.Gear.HIGH : Drive.Gear.LOW);
			break;
		case HOLD_RB_TO_PUSH:
			drive.setGear(driverGamepad.getButtonHeld(XboxController.RIGHT_BUMPER) ? Drive.Gear.LOW : Drive.Gear.HIGH);
			break;
		case TWO_BUTTON_TOGGLE:
			if (driverGamepad.getButtonOnce(XboxController.B_BUTTON)) {
				drive.setGear(Drive.Gear.HIGH);
			} else if (driverGamepad.getButtonOnce(XboxController.A_BUTTON)) {
				drive.setGear(Drive.Gear.LOW);
			}
			break;
		}
		if(pdp.getTotalCurrent() < 20) {
			drive.brownoutMonitor.updateBatteryVoltage();
		}
	}

	private enum ShiftMode {
		HOLD_B_TO_BOOST, HOLD_B_TO_PUSH, HOLD_RB_TO_BOOST, HOLD_RB_TO_PUSH, TWO_BUTTON_TOGGLE
	}

	private enum ControlMode {
		ARCADE, SPLIT_ARCADE, TANK, CHEESY, SINGLE_STICK_CHEESY, CHEESY_CAR, CAR
	}

	public void update() {
		if(this.isDisabled()){
			switchPrimaryStick = SmartDashboard.getBoolean("Switch Primary Stick", false);
			squareThrottle = SmartDashboard.getBoolean("Square Throttle", false);
			squareTurning = SmartDashboard.getBoolean("Square Turning", false);
			currentLimiting = SmartDashboard.getBoolean("Current Limiting", false);
			selectedNeutralMode = neutralModeChooser.getSelected();
			selectedControlMode = controlModeChooser.getSelected();
			selectedShiftMode = shiftModeChooser.getSelected();
			drive.setNeutralMode(selectedNeutralMode);
		}
	}
}
