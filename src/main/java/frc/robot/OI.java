package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * Stands for <b>Operator Interface</b>
 */
public class OI 
{
        public static final XboxController xboxController = new XboxController(frc.robot.Constants.OI.kXboxControllerPort);
        public static final JoystickButton A = new JoystickButton(xboxController, XboxController.Button.kA.value);
        public static final JoystickButton B = new JoystickButton(xboxController, XboxController.Button.kB.value);
        public static final JoystickButton X = new JoystickButton(xboxController, XboxController.Button.kX.value);
        public static final JoystickButton Y = new JoystickButton(xboxController, XboxController.Button.kY.value);
        public static final POVButton DPadUP = new POVButton(xboxController, 0);
        public static final POVButton DPadDOWN = new POVButton(xboxController, 180);
        public static final POVButton DpadRIGHT = new POVButton(xboxController,90);
        public static final POVButton DPadLEFT = new POVButton(xboxController, 270);
        public static final JoystickButton LeftBumper = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
        public static final JoystickButton RightBumper = new JoystickButton(xboxController, XboxController.Button.kRightBumper.value);

}
