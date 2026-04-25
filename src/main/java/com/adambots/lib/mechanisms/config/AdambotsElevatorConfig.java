package com.adambots.lib.mechanisms.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.adambots.lib.actuators.BaseMotor;
import com.adambots.lib.visualization.MechanismVisualizer;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Fluent configuration for {@link com.adambots.lib.mechanisms.AdambotsElevator}.
 *
 * <p>Translates to YAMS {@link SmartMotorControllerConfig} and {@link ElevatorConfig}
 * internally. Phoenix Pro features log a warning when used with non-Phoenix motors.
 */
public final class AdambotsElevatorConfig {

    // Motor control
    private Double kP, kI, kD;
    private int pidSlot = 0;
    private Double trapezoidCruiseRPS;
    private Double trapezoidAccelRPSps;
    private ElevatorFeedforward elevatorFeedforward;
    private Double statorCurrentAmps;
    private Double supplyCurrentAmps;
    private double gearRatio = 1.0;
    private Distance tolerance;
    private Boolean brakeMode = true;
    private Voltage voltageCompensation;

    // Phoenix Pro
    private boolean focEnabled = false;
    private Double expoKV, expoKA;

    // Elevator-specific
    private Distance drumRadius;
    private Mass mass;
    private Distance minHeight, maxHeight;
    private Distance startingHeight;
    private Integer cascadingStages;
    private boolean horizontal = false;
    private DCMotor simMotorOverride;

    // AdambotsLib
    private Subsystem subsystem;
    private String telemetryName;
    private TelemetryVerbosity telemetryVerbosity;
    private MechanismVisualizer visualizer;
    private int visualizerIndex = -1;
    private Translation3d visualizerBase = Translation3d.kZero;

    // ---------- Motor control ----------

    public AdambotsElevatorConfig withPID(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
        return this;
    }

    public AdambotsElevatorConfig withPIDSlot(int slot) {
        this.pidSlot = slot;
        return this;
    }

    public AdambotsElevatorConfig withMotionMagic(double cruiseRPS, double accelRPSps) {
        this.trapezoidCruiseRPS = cruiseRPS;
        this.trapezoidAccelRPSps = accelRPSps;
        return this;
    }

    /** Elevator feedforward gains (kS, kG, kV, kA). */
    public AdambotsElevatorConfig withFeedforward(double kS, double kG, double kV, double kA) {
        this.elevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        return this;
    }

    public AdambotsElevatorConfig withCurrentLimits(double statorAmps, double supplyAmps) {
        this.statorCurrentAmps = statorAmps;
        this.supplyCurrentAmps = supplyAmps;
        return this;
    }

    public AdambotsElevatorConfig withGearRatio(double reductionRatio) {
        this.gearRatio = reductionRatio;
        return this;
    }

    public AdambotsElevatorConfig withTolerance(Distance tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public AdambotsElevatorConfig withBrakeMode(boolean brake) {
        this.brakeMode = brake;
        return this;
    }

    public AdambotsElevatorConfig withVoltageCompensation(Voltage nominal) {
        this.voltageCompensation = nominal;
        return this;
    }

    // ---------- Phoenix Pro ----------

    public AdambotsElevatorConfig withFOC(boolean enable) {
        this.focEnabled = enable;
        return this;
    }

    public AdambotsElevatorConfig withMotionMagicExpo(double expoKV, double expoKA) {
        this.expoKV = expoKV;
        this.expoKA = expoKA;
        return this;
    }

    // ---------- Elevator-specific ----------

    public AdambotsElevatorConfig withDrumRadius(Distance radius) {
        this.drumRadius = radius;
        return this;
    }

    public AdambotsElevatorConfig withMass(Mass mass) {
        this.mass = mass;
        return this;
    }

    public AdambotsElevatorConfig withMass(double kilograms) {
        this.mass = Kilograms.of(kilograms);
        return this;
    }

    public AdambotsElevatorConfig withHeightRange(Distance min, Distance max) {
        this.minHeight = min;
        this.maxHeight = max;
        return this;
    }

    public AdambotsElevatorConfig withStartingHeight(Distance height) {
        this.startingHeight = height;
        return this;
    }

    /** Number of cascading stages (for multi-stage elevators). */
    public AdambotsElevatorConfig withCascadingStages(int stages) {
        this.cascadingStages = stages;
        return this;
    }

    /** Marks this elevator as horizontal (gravity does not apply along travel axis). */
    public AdambotsElevatorConfig horizontal() {
        this.horizontal = true;
        return this;
    }

    public AdambotsElevatorConfig withSimMotor(DCMotor motor) {
        this.simMotorOverride = motor;
        return this;
    }

    // ---------- AdambotsLib ----------

    public AdambotsElevatorConfig withSubsystem(Subsystem subsystem) {
        this.subsystem = subsystem;
        return this;
    }

    public AdambotsElevatorConfig withTelemetry(String name, TelemetryVerbosity verbosity) {
        this.telemetryName = name;
        this.telemetryVerbosity = verbosity;
        return this;
    }

    public AdambotsElevatorConfig withVisualizer(MechanismVisualizer visualizer,
                                                   int index,
                                                   Translation3d baseOffset) {
        this.visualizer = visualizer;
        this.visualizerIndex = index;
        this.visualizerBase = baseOffset;
        return this;
    }

    public AdambotsElevatorConfig withVisualizer(MechanismVisualizer visualizer, int index) {
        return withVisualizer(visualizer, index, Translation3d.kZero);
    }

    // ---------- Build ----------

    public void validateAgainst(BaseMotor motor) {
        MotorConfigHelpers.warnProFeaturesOnNonPhoenix(
            motor,
            "AdambotsElevator[" + (telemetryName != null ? telemetryName : "unnamed") + "]",
            focEnabled,
            expoKV != null || expoKA != null);
    }

    public SmartMotorControllerConfig buildMotorConfig() {
        SmartMotorControllerConfig cfg = subsystem != null
            ? new SmartMotorControllerConfig(subsystem)
            : new SmartMotorControllerConfig();

        if (kP != null) {
            cfg = cfg.withClosedLoopController(kP, kI, kD, MotorConfigHelpers.slotOf(pidSlot));
        }

        if (trapezoidCruiseRPS != null && trapezoidAccelRPSps != null) {
            cfg = cfg.withTrapezoidalProfile(
                RotationsPerSecond.of(trapezoidCruiseRPS),
                RotationsPerSecondPerSecond.of(trapezoidAccelRPSps));
        }

        if (expoKV != null && expoKA != null) {
            Voltage v = voltageCompensation != null ? voltageCompensation : Volts.of(12);
            double volts = v.in(Volts);
            cfg = cfg.withExponentialProfile(
                v,
                RotationsPerSecond.of(volts / expoKV),
                RotationsPerSecondPerSecond.of(volts / expoKA));
        }

        if (elevatorFeedforward != null) {
            cfg = cfg.withFeedforward(elevatorFeedforward, MotorConfigHelpers.slotOf(pidSlot));
        }

        cfg = cfg.withGearing(gearRatio);
        if (statorCurrentAmps != null) cfg = cfg.withStatorCurrentLimit(Amps.of(statorCurrentAmps));
        if (supplyCurrentAmps != null) cfg = cfg.withSupplyCurrentLimit(Amps.of(supplyCurrentAmps));
        if (tolerance != null) cfg = cfg.withClosedLoopTolerance(tolerance);
        if (brakeMode != null) {
            cfg = cfg.withIdleMode(brakeMode
                ? SmartMotorControllerConfig.MotorMode.BRAKE
                : SmartMotorControllerConfig.MotorMode.COAST);
        }
        if (voltageCompensation != null) cfg = cfg.withVoltageCompensation(voltageCompensation);

        if (minHeight != null && maxHeight != null) {
            cfg = cfg.withSoftLimit(minHeight, maxHeight);
        }

        if (telemetryName != null && telemetryVerbosity != null) {
            cfg = cfg.withTelemetry(telemetryName, telemetryVerbosity);
        }

        return cfg;
    }

    public ElevatorConfig buildYamsConfig(SmartMotorController smartMotor) {
        ElevatorConfig eCfg = new ElevatorConfig(smartMotor);
        if (drumRadius != null) eCfg = eCfg.withDrumRadius(drumRadius);
        if (mass != null) eCfg = eCfg.withMass(mass);
        if (minHeight != null && maxHeight != null) eCfg = eCfg.withSoftLimits(minHeight, maxHeight);
        if (startingHeight != null) eCfg = eCfg.withStartingHeight(startingHeight);
        if (cascadingStages != null) eCfg = eCfg.withCascadingElevatorStages(cascadingStages);
        if (horizontal) eCfg = eCfg.withHorizontalElevator();
        if (telemetryName != null && telemetryVerbosity != null) {
            eCfg = eCfg.withTelemetry(telemetryName, telemetryVerbosity);
        }
        return eCfg;
    }

    public DCMotor getSimMotorOverride() { return simMotorOverride; }
    public MechanismVisualizer getVisualizer() { return visualizer; }
    public int getVisualizerIndex() { return visualizerIndex; }
    public Translation3d getVisualizerBase() { return visualizerBase; }

    /** True if {@code withFOC(true)} was requested. Honored on Phoenix motors. */
    public boolean isFocEnabled() { return focEnabled; }
}
