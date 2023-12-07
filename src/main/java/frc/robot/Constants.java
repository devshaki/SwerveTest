// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Swerve {

    public static class PID {
      // ! --- DO NOT USE THESE PID k VARIABLES IN PRODUCTION! I DID NOT TEST THEM YET ------------------ 
      public static class Drive {
        /**
         * Static Friction Offset (to overcome the friction of the system)
         */
        public static final double kS = 2.0;
        /**
         * Velocity Feedforward (to continue the current speed)
         */
        public static final double kV = 2.0;
        /**
         * the voltage needed to reach a certain excalaration (i have no idea what number to put)
         */
        public static final double kA = 0;

        /**
         * Proportional tuning - error
         */
        public static final double kP = 2.0;
        /**
         * Integral tuning - learning
         */
        public static final double kI = 2.0;
        /**
         * Derivative tuning - overshoot
         */
        public static final double kD = 2.0;
      } 

      public static class Steer {
        /**
         * Static Friction Offset (to overcome the friction of the system)
         */
        public static final double kS = 2.0;
        /**
         * Velocity Feedforward (to continue the current speed)
         */
        public static final double kV = 2.0;
        /**
         * the voltage needed to reach a certain excalaration (i have no idea what number to put)
         */
        public static final double kA = 0;


        /**
         * Proportional tuning - error
         */
        public static final double kP = 2.0;
        /**
         * Integral tuning - learning
         */
        public static final double kI = 2.0;
        /**
         * Derivative tuning - overshoot
         */
        public static final double kD = 2.0;
      } 

    }

    public static class Stats {
      public static final double kMaxVoltage = 12.0;
      public static final double kStatorCurrentLimit = 35.0;
      public static final double kSupplyCurrentLimit = 35.0;
      
      /**
       * Distance between the center of the right wheels to the center of the left wheels (Meters)
       */
      public static final double kTrackWidthMeters = 85.5;

      /**
       * Distance between the center of the back wheels to the center of the front wheels (Meters)
       */
      public static final double kWheelbaseMeters = 85.5;

      


      /**
       * The ratio between the Motor and the center wheel of the Swerve module (which the CANcoder lies on)
       */
      public static final double kRotorToSensorRatio = 8.14;

      public static final double kDriveWheelRadiusInches = 2;
      
      
    }
  }

  public static class Drive {
      
    public static class Stats {
        /**
         * Distance between the center of the right wheels to the center of the left wheels (Meters)
         */
        public static final double kTrackWidthMeters = 85.5;

        /**
         * Distance between the center of the back wheels to the center of the front wheels (Meters)
         */
        public static final double kWheelbaseMeters = 85.5;

        /**
         * The module offset relative to the robot's base in degrees
         */
        public static final double kFrontLeftModuleOffsetInDegrees = 34.5;
        /**
         * The module offset relative to the robot's base in degrees
         */
        public static final double kFrontRightModuleOffsetInDegrees = 34.5;
        /**
         * The module offset relative to the robot's base in degrees
         */
        public static final double kBackLeftModuleOffsetInDegrees = 34.5;
        /**
         * The module offset relative to the robot's base in degrees
         */
        public static final double kBackRightModuleOffsetInDegrees = 34.5;

    }

    public static class Motors {
      public static final int kFrontLeftDriveFalconCANID = 1;
      public static final int kFrontLeftSteerFalconCANID = 2;

      public static final int kFrontRightDriveFalconCANID = 3;
      public static final int kFrontRightSteerFalconCANID = 4;

      public static final int kBackLeftDriveFalconCANID = 5;
      public static final int kBackLeftSteerFalconCANID = 6;

      public static final int kBackRightDriveFalconCANID = 7;
      public static final int kBackRightSteerFalconCANID = 8;


    }

    public static class Encoders {
      // ? Only the steer encoder exists (seperate from the encoder inside of the Falcon 500 because of ratio problems between the wheels of the swerve modules)
      public static final int kFrontLeftSteerEncoderCANID = 9;
      public static final int kFrontRightSteerEncoderCANID = 10;
      public static final int kBackLeftSteerEncoderCANID = 11;
      public static final int kBackRightSteerEncoderCANID = 12;
    }

  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class OI
  {
    public static final int kXboxControllerPort = 0;
    public static final double kXboxcontrollerDrift = 0.1;
  }
  public static class FieldConstants {
    // ! TODO CHANGE THIS TO ACTUAL FIELD LENGTH FOR 2024
    public static final double fieldLength = 0.0;
    public static final double fieldWidth = 0.0;
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);
  }
  
}