"use client";

import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Plane, Camera, Target, MapPin, Crosshair } from "lucide-react";
import type { UseMQTTReturn, ArUcoDetection } from "@/hooks/use-mqtt";

interface ControlPanelProps {
    mqtt: UseMQTTReturn | null;
    arUcoDetection: ArUcoDetection | null;
}

export function ControlPanel({ mqtt, arUcoDetection }: ControlPanelProps) {
    const [isArmed, setIsArmed] = useState(false);
    const [isTakingOff, setIsTakingOff] = useState(false);
    const [activeLandingMode, setActiveLandingMode] = useState<
        "manual" | "auto" | null
    >(null);
    const [centeringEnabled, setCenteringEnabled] = useState(false);
    const [altitude, setAltitude] = useState<string>("5.0");
    const [destination, setDestination] = useState<string>("");

    const isConnected = mqtt?.isConnected ?? false;
    const arUcoDetected = arUcoDetection?.detected ?? false;

    /**
     * Handle takeoff command
     */
    const handleTakeoff = async () => {
        if (!mqtt) return;

        const altitudeValue = parseFloat(altitude) || 5.0;
        setIsTakingOff(true);

        const success = await mqtt.sendTakeoff(altitudeValue);

        if (success) {
            setIsArmed(true);
            console.log(`✓ Takeoff command sent (altitude: ${altitudeValue}m)`);
        } else {
            console.error("✗ Failed to send takeoff command");
        }

        setTimeout(() => setIsTakingOff(false), 2000);
    };

    /**
     * Handle manual landing toggle
     */
    const handleManualLandingToggle = async () => {
        if (!mqtt) return;

        if (activeLandingMode === "manual") {
            setActiveLandingMode(null);
            console.log("✓ Manual landing cancelled");
            return;
        }

        setActiveLandingMode("manual");
        const success = await mqtt.sendLand();

        if (success) {
            console.log("✓ Manual landing command sent");
            setTimeout(() => {
                setActiveLandingMode(null);
                setIsArmed(false);
            }, 10000);
        } else {
            console.error("✗ Failed to send land command");
            setActiveLandingMode(null);
        }
    };

    const handleAutoLandingToggle = async () => {
        if (!mqtt) return;

        if (activeLandingMode === "auto") {
            setActiveLandingMode(null);
            console.log("✓ Auto-landing cancelled");
            return;
        }

        if (!arUcoDetected) {
            alert("⚠️ No ArUco marker detected. Cannot initiate auto-land.");
            return;
        }

        setActiveLandingMode("auto");
        const success = await mqtt.sendAutoLand();

        if (success) {
            console.log("✓ Auto-land command sent");
            setTimeout(() => {
                setActiveLandingMode(null);
                setIsArmed(false);
            }, 15000);
        } else {
            console.error("✗ Failed to send auto-land command");
            setActiveLandingMode(null);
        }
    };

    const handleToggleArm = async () => {
        if (!mqtt) return;

        const success = isArmed
            ? await mqtt.sendDisarm()
            : await mqtt.sendArm();

        if (success) {
            setIsArmed(!isArmed);
            console.log(`✓ Drone ${!isArmed ? "armed" : "disarmed"}`);
        } else {
            console.error("✗ Failed to toggle arm state");
        }
    };

    const handleMove = async () => {
        if (!mqtt || !destination.trim()) return;

        const success = await mqtt.sendMove(destination.trim());

        if (success) {
            console.log(`✓ Move command sent (destination: ${destination})`);
            setDestination("");
        } else {
            console.error("✗ Failed to send move command");
        }
    };

    /**
     * Handle enable/disable centering
     */
    const handleToggleCentering = async () => {
        if (!mqtt) return;

        const success = centeringEnabled
            ? await mqtt.sendDisableCentering()
            : await mqtt.sendEnableCentering();

        if (success) {
            setCenteringEnabled(!centeringEnabled);
            console.log(
                `✓ Centering ${centeringEnabled ? "disabled" : "enabled"}`
            );
        } else {
            console.error("✗ Failed to toggle centering");
        }
    };

    const handleEmergencyStop = async () => {
        if (!mqtt) return;

        const success = await mqtt.sendEmergencyStop();

        if (success) {
            setIsArmed(false);
            setIsTakingOff(false);
            setActiveLandingMode(null);
            setCenteringEnabled(false);
            console.log("⚠️ EMERGENCY STOP - All operations halted");
        } else {
            console.error("✗ Failed to send emergency stop");
        }
    };

    return (
        <Card className="p-6 bg-slate-950 border-slate-800">
            <div className="space-y-6">
                <div className="flex items-center justify-between">
                    <div>
                        <h2 className="text-xl font-bold text-slate-200 uppercase tracking-wide">
                            Mission Control
                        </h2>
                        <p className="text-sm text-slate-500 mt-1">
                            Los Angeles City College Drone Operations
                        </p>
                    </div>
                    <div className="flex items-center gap-2">
                        <Badge
                            variant={isArmed ? "destructive" : "secondary"}
                            className="uppercase"
                        >
                            {isArmed ? "Armed" : "Disarmed"}
                        </Badge>
                        {centeringEnabled && (
                            <Badge variant="default" className="uppercase">
                                <Crosshair className="mr-1 h-3 w-3" />
                                Centering
                            </Badge>
                        )}
                    </div>
                </div>

                {!isConnected && (
                    <div className="p-3 bg-amber-950 border border-amber-800 rounded-lg">
                        <p className="text-sm text-amber-200">
                            ⚠️ Not connected to MQTT broker
                        </p>
                    </div>
                )}

                {/* Manual Arm/Disarm */}
                <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                    <h3 className="text-sm font-semibold text-slate-400 uppercase tracking-wide mb-3">
                        System Status
                    </h3>
                    <Button
                        onClick={handleToggleArm}
                        disabled={!isConnected}
                        className={`w-full h-12 disabled:opacity-50 ${
                            isArmed
                                ? "bg-red-600 hover:bg-red-700"
                                : "bg-slate-600 hover:bg-slate-700"
                        }`}
                    >
                        <div className="flex flex-col items-center w-full">
                            <span className="font-semibold">
                                {isArmed ? "DISARM DRONE" : "ARM DRONE"}
                            </span>
                            <span className="text-xs opacity-80">
                                {isArmed
                                    ? "Enable landing controls"
                                    : "Manual arm for testing"}
                            </span>
                        </div>
                    </Button>
                </div>

                {/* Takeoff Section */}
                <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                    <h3 className="text-sm font-semibold text-slate-400 uppercase tracking-wide mb-3">
                        Takeoff
                    </h3>
                    <div className="space-y-3">
                        <div>
                            <label className="text-xs text-slate-400 block mb-2">
                                Altitude (meters)
                            </label>
                            <input
                                type="number"
                                value={altitude}
                                onChange={(e) => setAltitude(e.target.value)}
                                min="1"
                                max="50"
                                step="0.5"
                                disabled={!isConnected || isArmed}
                                className="w-full px-3 py-2 bg-slate-800 border border-slate-700 rounded text-slate-200 text-sm disabled:opacity-50"
                            />
                        </div>
                        <Button
                            onClick={handleTakeoff}
                            disabled={!isConnected || isArmed || isTakingOff}
                            className="w-full bg-emerald-600 hover:bg-emerald-700 disabled:opacity-50 h-12"
                        >
                            <Target className="mr-2 h-5 w-5" />
                            <div className="flex flex-col items-start">
                                <span className="font-semibold">
                                    {isTakingOff ? "Taking Off..." : "Takeoff"}
                                </span>
                                <span className="text-xs opacity-80">
                                    Arm & launch drone
                                </span>
                            </div>
                        </Button>
                    </div>
                </div>

                {/* Move Section */}
                <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                    <h3 className="text-sm font-semibold text-slate-400 uppercase tracking-wide mb-3">
                        Navigation
                    </h3>
                    <div className="space-y-3">
                        <div>
                            <label className="text-xs text-slate-400 block mb-2">
                                Destination
                            </label>
                            <input
                                type="text"
                                value={destination}
                                onChange={(e) => setDestination(e.target.value)}
                                placeholder="Enter destination..."
                                disabled={!isConnected || !isArmed}
                                className="w-full px-3 py-2 bg-slate-800 border border-slate-700 rounded text-slate-200 text-sm disabled:opacity-50"
                            />
                        </div>
                        <Button
                            onClick={handleMove}
                            disabled={
                                !isConnected || !isArmed || !destination.trim()
                            }
                            className="w-full bg-blue-600 hover:bg-blue-700 disabled:opacity-50"
                        >
                            <MapPin className="mr-2 h-4 w-4" />
                            Move to Destination
                        </Button>
                    </div>
                </div>

                {/* Landing Controls */}
                <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                    <h3 className="text-sm font-semibold text-slate-400 uppercase tracking-wide mb-3">
                        Landing Controls
                    </h3>
                    <div className="space-y-3">
                        <Button
                            onClick={handleManualLandingToggle}
                            disabled={
                                !isConnected ||
                                (activeLandingMode !== null &&
                                    activeLandingMode !== "manual")
                            }
                            className={`w-full h-12 disabled:opacity-50 ${
                                activeLandingMode === "manual"
                                    ? "bg-orange-600 hover:bg-orange-700 animate-pulse"
                                    : "bg-sky-600 hover:bg-sky-700"
                            }`}
                        >
                            <Plane className="mr-2 h-5 w-5" />
                            <div className="flex flex-col items-start">
                                <span className="font-semibold">
                                    {activeLandingMode === "manual"
                                        ? "Cancel Manual Landing"
                                        : "Manual Landing"}
                                </span>
                                <span className="text-xs opacity-80">
                                    {activeLandingMode === "manual"
                                        ? "Click to cancel"
                                        : "Send land command"}
                                </span>
                            </div>
                        </Button>

                        <Button
                            onClick={handleAutoLandingToggle}
                            disabled={
                                !isConnected ||
                                (activeLandingMode !== null &&
                                    activeLandingMode !== "auto") ||
                                (activeLandingMode !== "auto" && !arUcoDetected)
                            }
                            className={`w-full h-12 disabled:opacity-50 ${
                                activeLandingMode === "auto"
                                    ? "bg-orange-600 hover:bg-orange-700 animate-pulse"
                                    : "bg-purple-600 hover:bg-purple-700"
                            }`}
                        >
                            <Camera className="mr-2 h-5 w-5" />
                            <div className="flex flex-col items-start">
                                <span className="font-semibold">
                                    {activeLandingMode === "auto"
                                        ? "Cancel Auto Landing"
                                        : "Auto Landing - CV"}
                                </span>
                                <span className="text-xs opacity-80">
                                    {activeLandingMode === "auto"
                                        ? "Click to cancel"
                                        : arUcoDetected
                                        ? "Marker detected ✓"
                                        : "No marker detected"}
                                </span>
                            </div>
                        </Button>
                    </div>
                </div>

                {/* Centering Control */}
                <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                    <h3 className="text-sm font-semibold text-slate-400 uppercase tracking-wide mb-3">
                        Marker Centering
                    </h3>
                    <Button
                        onClick={handleToggleCentering}
                        disabled={!isConnected}
                        className={`w-full h-12 disabled:opacity-50 ${
                            centeringEnabled
                                ? "bg-orange-600 hover:bg-orange-700"
                                : "bg-emerald-600 hover:bg-emerald-700"
                        }`}
                    >
                        <Crosshair className="mr-2 h-5 w-5" />
                        <div className="flex flex-col items-start">
                            <span className="font-semibold">
                                {centeringEnabled ? "Disable" : "Enable"}{" "}
                                Centering
                            </span>
                            <span className="text-xs opacity-80">
                                {centeringEnabled ? "Stop" : "Start"} marker
                                tracking
                            </span>
                        </div>
                    </Button>
                </div>

                {(isTakingOff || activeLandingMode !== null) && (
                    <div className="p-3 bg-amber-950 border border-amber-800 rounded-lg">
                        <p className="text-sm text-amber-200 font-medium">
                            ⚠️ Operation in progress:{" "}
                            <span className="uppercase font-bold">
                                {isTakingOff
                                    ? "Takeoff"
                                    : activeLandingMode === "auto"
                                    ? "Auto-Landing"
                                    : "Manual Landing"}
                            </span>
                        </p>
                    </div>
                )}

                {/* Emergency Stop */}
                <Button
                    variant="destructive"
                    className="w-full h-14 text-lg font-bold uppercase tracking-wider bg-red-600 hover:bg-red-700"
                    onClick={handleEmergencyStop}
                >
                    Emergency Stop
                </Button>
            </div>
        </Card>
    );
}
