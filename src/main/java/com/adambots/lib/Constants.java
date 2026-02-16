package com.adambots.lib;

import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

/**
 * All constant values for robot operation - Any ports should be defined in {@link RobotMap} 
 */
public final class Constants {
    public static final String kDefaultShuffleboardTab = "debug";
    public static Field2d odomField = new Field2d();    
    public static final Boolean enableAutomaticShuffleboardRecording = false;

    public static final class LEDConstants {
        public static final int LEDS_IN_STRIP = 100;
        public static final StripTypeValue LED_STRIP_TYPE = StripTypeValue.GRB; // if this is not set properly, the colors will not work

        public static final Color off = new Color(0, 0, 0);
        public static final Color adambotsYellow = new Color(255, 255, 0);
        public static final Color yellow = new Color(255, 255, 0);
        public static final Color blue = new Color(0, 0, 255);
        public static final Color orange = new Color(180, 90, 5);
        public static final Color pink = new Color(255, 200, 200);
        public static final Color purple = new Color(255, 0, 255);
        public static final Color red = new Color(255, 150, 0);
        public static final Color green = new Color(0, 255, 0);
        public static final Color white = new Color(255, 255, 255);
    }

    public static final class VisionConstants {
        public static final String defaultAprilLimelite = "limelight-aprilgg";
    }

    public static final class DriveConstants {
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kRearLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kRearRightDriveMotorReversed = true;

        // Distance between centers of right and left wheels on robot
        public static final Distance kTrackWidth = Meters.of(0.61);
        // Distance between front and back wheels on robot
        public static final Distance kWheelBase = Meters.of(0.61);
        // Drive base radius. Distance from robot center to furthest module, hypotenuse of kTrackWidth/2 and kWheelBase/2
        public static final Distance kDrivebaseRadius = Meters.of(Math.hypot(kTrackWidth.in(Meters)/2, kWheelBase.in(Meters)/2));

        public enum ModulePosition {
            FRONT_LEFT,
            FRONT_RIGHT,
            REAR_LEFT,
            REAR_RIGHT
        }

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase.in(Meters) / 2, kTrackWidth.in(Meters) / 2),
            new Translation2d(kWheelBase.in(Meters) / 2, -kTrackWidth.in(Meters) / 2),
            new Translation2d(-kWheelBase.in(Meters) / 2, kTrackWidth.in(Meters) / 2),
            new Translation2d(-kWheelBase.in(Meters) / 2, -kTrackWidth.in(Meters) / 2)
        );

        // Xbox controller joystick deadzone
        public static final double kDeadZone = 0.0;

        //Max speed of the robot, used in teleop and auton (should be set to real world value)
        //Drive the robot on carpet and measure the speed with a stopwatch
        //When FOC is enabled - 17.1 ft/s × 0.3048 m/ft = 5.21 m/s. When FOC is disabled - 17.7 ft/s × 0.3048 m/ft = 5.40 m/s. [ 1 foot = 0.3048 meters ]
        // Previous year's value 4.35;
        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(5.21);

        //Rotational speed factor of the robot to be used for the teleop drive command
        public static final AngularVelocity kTeleopRotationalSpeed = RadiansPerSecond.of(10);
    }

    public static final class ModuleConstants {
        public static final double kMK4L1GearRatio = 1/8.14; // 1:8.14 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909
        public static final double kMK4L2GearRatio = 1/6.75; // 1:6.75 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909
        public static final double kMK4L3GearRatio = 1/6.12; // 1:6.12 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909

        // The 2025 Robot will use an Adapter Kit to work with Kraken. This changes the Gear Ratios to L1+, L2+ and L3+
        public static final double kMK4IL1PlusGearRatio = 1/7.13; // 1:7.13 as per https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i?variant=47576386502957
        public static final double kMK4IL2PlusGearRatio = 1/5.9; // 1:5.9 as per https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i?variant=47576386502957
        public static final double kMK4IL3PlusGearRatio = 1/5.36; // 1:5.36 as per https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i?variant=47576386502957

        //Define gear ratio as motor revolutions per wheel rotation
        public static final double kSteeringGearRatio = 150.0/7.0; // 150/7:1 as per https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033732909

        public static final Current kDriveCurrentLimit = Amps.of(32); //Current limit of drive motors, higher values mean faster acceleration but lower battery life
        public static final Current kTurningCurrentLimit = Amps.of(21); //Current limit of turning motors
        public static final Voltage kNominalVoltage = Volts.of(12.6); //Nominal battery voltage for motor voltage compensation

        public static final Distance kWheelRadius = Meters.of(0.0478); //0.047625 //Should be as precise as you can get it
        public static final double kSwerveModuleFinalGearRatio = kMK4IL2PlusGearRatio; //Google the swerve module model to find this value

        // Convert drive motor rpm to linear wheel speed                  Motor RPM to Wheel RPM -> RPM to rad/s -> Wheel rad/s to linear m/s
        public static final double kDriveEncoderVelocityConversionFactor = kSwerveModuleFinalGearRatio * (Math.PI/30) * kWheelRadius.in(Meters);

        // Convert drive motor rotations to linear distance             Motor rot to Wheel rot -> Wheel rot to linear meters (circumference)
        public static final double kDriveEncoderPositionConversionFactor = kSwerveModuleFinalGearRatio * 2*Math.PI * kWheelRadius.in(Meters);

        public static final double kPModuleTurningController = 0.7; //PID Values for turning motors
        // public static final double kDModuleTurningController = 0.026;
        public static final double kDModuleTurningController = 0;
    }

}