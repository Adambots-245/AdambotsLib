package com.adambots.lib.mechanisms.config;

import static edu.wpi.first.units.Units.Amps;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Fluent configuration for {@link com.adambots.lib.mechanisms.AdambotsFlyWheel}.
 * Translates to YAMS {@link FlyWheelConfig} internally.
 */
public final class AdambotsFlyWheelConfig {

    // Motor control
    private Double kP, kI, kD;
    private int pidSlot = 0;
    private SimpleMotorFeedforward feedforward;
    private Double statorCurrentAmps;
    private Double supplyCurrentAmps;
    private double gearRatio = 1.0;
    private Boolean brakeMode = false; // default coast for flywheels
    private Voltage voltageCompensation;

    // Phoenix Pro
    private boolean focEnabled = false;
    private Double expoKV, expoKA;

    // FlyWheel-specific
    private Distance wheelDiameter;
    private Mass mass;
    private Double momentOfInertia;
    private AngularVelocity minSpeed, maxSpeed;
    private DCMotor simMotorOverride;

    // AdambotsLib
    private Subsystem subsystem;
    private String telemetryName;
    private TelemetryVerbosity telemetryVerbosity;
    private MechanismVisualizer visualizer;
    private int visualizerIndex = -1;
    private Translation3d visualizerOffset = Translation3d.kZero;

    // ---------- Motor control ----------

    public AdambotsFlyWheelConfig withPID(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        return this;
    }

    public AdambotsFlyWheelConfig withPIDSlot(int slot) {
        this.pidSlot = slot;
        return this;
    }

    /** Simple velocity feedforward (kS, kV, kA). */
    public AdambotsFlyWheelConfig withFeedforward(double kS, double kV, double kA) {
        this.feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        return this;
    }

    public AdambotsFlyWheelConfig withCurrentLimits(double statorAmps, double supplyAmps) {
        this.statorCurrentAmps = statorAmps;
        this.supplyCurrentAmps = supplyAmps;
        return this;
    }

    public AdambotsFlyWheelConfig withGearRatio(double reductionRatio) {
        this.gearRatio = reductionRatio;
        return this;
    }

    public AdambotsFlyWheelConfig withBrakeMode(boolean brake) {
        this.brakeMode = brake;
        return this;
    }

    public AdambotsFlyWheelConfig withVoltageCompensation(Voltage nominal) {
        this.voltageCompensation = nominal;
        return this;
    }

    // ---------- Phoenix Pro ----------

    public AdambotsFlyWheelConfig withFOC(boolean enable) {
        this.focEnabled = enable;
        return this;
    }

    public AdambotsFlyWheelConfig withMotionMagicExpo(double expoKV, double expoKA) {
        this.expoKV = expoKV;
        this.expoKA = expoKA;
        return this;
    }

    // ---------- FlyWheel-specific ----------

    public AdambotsFlyWheelConfig withWheelDiameter(Distance diameter) {
        this.wheelDiameter = diameter;
        return this;
    }

    public AdambotsFlyWheelConfig withMass(Mass mass) {
        this.mass = mass;
        return this;
    }

    public AdambotsFlyWheelConfig withMomentOfInertia(double kgm2) {
        this.momentOfInertia = kgm2;
        return this;
    }

    public AdambotsFlyWheelConfig withSpeedRange(AngularVelocity min, AngularVelocity max) {
        this.minSpeed = min;
        this.maxSpeed = max;
        return this;
    }

    public AdambotsFlyWheelConfig withSimMotor(DCMotor motor) {
        this.simMotorOverride = motor;
        return this;
    }

    // ---------- AdambotsLib ----------

    public AdambotsFlyWheelConfig withSubsystem(Subsystem subsystem) {
        this.subsystem = subsystem;
        return this;
    }

    public AdambotsFlyWheelConfig withTelemetry(String name, TelemetryVerbosity verbosity) {
        this.telemetryName = name;
        this.telemetryVerbosity = verbosity;
        return this;
    }

    public AdambotsFlyWheelConfig withVisualizer(MechanismVisualizer visualizer,
                                                   int index,
                                                   Translation3d offset) {
        this.visualizer = visualizer;
        this.visualizerIndex = index;
        this.visualizerOffset = offset;
        return this;
    }

    public AdambotsFlyWheelConfig withVisualizer(MechanismVisualizer visualizer, int index) {
        return withVisualizer(visualizer, index, Translation3d.kZero);
    }

    // ---------- Build ----------

    public void validateAgainst(BaseMotor motor) {
        MotorConfigHelpers.warnProFeaturesOnNonPhoenix(
            motor,
            "AdambotsFlyWheel[" + (telemetryName != null ? telemetryName : "unnamed") + "]",
            focEnabled,
            expoKV != null || expoKA != null);
    }

    public SmartMotorControllerConfig buildMotorConfig() {
        SmartMotorControllerConfig cfg = subsystem != null
            ? new SmartMotorControllerConfig(subsystem)
            : new SmartMotorControllerConfig();

        if (kP != null) {
            ClosedLoopControllerSlot slotEnum = MotorConfigHelpers.slotOf(pidSlot);
            cfg = cfg.withClosedLoopController(kP, kI, kD, slotEnum);
        }

        if (feedforward != null) {
            cfg = cfg.withFeedforward(feedforward, MotorConfigHelpers.slotOf(pidSlot));
        }

        cfg = cfg.withGearing(gearRatio);
        if (statorCurrentAmps != null) cfg = cfg.withStatorCurrentLimit(Amps.of(statorCurrentAmps));
        if (supplyCurrentAmps != null) cfg = cfg.withSupplyCurrentLimit(Amps.of(supplyCurrentAmps));
        if (brakeMode != null) {
            cfg = cfg.withIdleMode(brakeMode
                ? SmartMotorControllerConfig.MotorMode.BRAKE
                : SmartMotorControllerConfig.MotorMode.COAST);
        }
        if (voltageCompensation != null) cfg = cfg.withVoltageCompensation(voltageCompensation);

        if (telemetryName != null && telemetryVerbosity != null) {
            cfg = cfg.withTelemetry(telemetryName, telemetryVerbosity);
        }
        return cfg;
    }

    public FlyWheelConfig buildYamsConfig(SmartMotorController smartMotor) {
        FlyWheelConfig fCfg = new FlyWheelConfig(smartMotor);
        if (wheelDiameter != null) fCfg = fCfg.withDiameter(wheelDiameter);
        if (mass != null) fCfg = fCfg.withMass(mass);
        if (momentOfInertia != null) fCfg = fCfg.withMOI(momentOfInertia);
        if (minSpeed != null && maxSpeed != null) fCfg = fCfg.withSoftLimit(minSpeed, maxSpeed);
        if (telemetryName != null && telemetryVerbosity != null) {
            fCfg = fCfg.withTelemetry(telemetryName, telemetryVerbosity);
        }
        return fCfg;
    }

    public DCMotor getSimMotorOverride() { return simMotorOverride; }
    public MechanismVisualizer getVisualizer() { return visualizer; }
    public int getVisualizerIndex() { return visualizerIndex; }
    public Translation3d getVisualizerOffset() { return visualizerOffset; }
}
