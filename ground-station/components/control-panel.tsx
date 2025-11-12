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
    const [isLanding, setIsLanding] = useState(false);
    const [isAutoLanding, setIsAutoLanding] = useState(false);
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
     * Handle land command
     */
    const handleLand = async () => {
        if (!mqtt) return;

        setIsLanding(true);
        const success = await mqtt.sendLand();

        if (success) {
            console.log("✓ Land command sent");
            setTimeout(() => setIsArmed(false), 3000);
        } else {
            console.error("✗ Failed to send land command");
        }

        setTimeout(() => setIsLanding(false), 2000);
    };

    /**
     * Handle move command
     */
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
     * Handle auto-land on ArUco marker
     */
    const handleAutoLand = async () => {
        if (!mqtt) return;

        if (!arUcoDetected) {
            alert("⚠️ No ArUco marker detected. Cannot initiate auto-land.");
            return;
        }

        setIsAutoLanding(true);
        const success = await mqtt.sendAutoLand();

        if (success) {
            console.log("✓ Auto-land command sent");
        } else {
            console.error("✗ Failed to send auto-land command");
        }

        setTimeout(() => {
            setIsAutoLanding(false);
            setIsArmed(false);
        }, 3000);
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

    /**
     * Handle emergency stop
     */
    const handleEmergencyStop = () => {
        setIsArmed(false);
        setIsTakingOff(false);
        setIsLanding(false);
        setIsAutoLanding(false);
        console.log("⚠️ EMERGENCY STOP - All operations halted");
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
                            onClick={handleLand}
                            disabled={!isConnected || !isArmed || isLanding}
                            className="w-full bg-sky-600 hover:bg-sky-700 disabled:opacity-50 h-12"
                        >
                            <Plane className="mr-2 h-5 w-5" />
                            <div className="flex flex-col items-start">
                                <span className="font-semibold">
                                    {isLanding
                                        ? "Landing..."
                                        : "Manual Landing"}
                                </span>
                                <span className="text-xs opacity-80">
                                    Standard descent
                                </span>
                            </div>
                        </Button>

                        <Button
                            onClick={handleAutoLand}
                            disabled={
                                !isConnected ||
                                !isArmed ||
                                !arUcoDetected ||
                                isAutoLanding
                            }
                            className="w-full bg-purple-600 hover:bg-purple-700 disabled:opacity-50 h-12"
                        >
                            <Camera className="mr-2 h-5 w-5" />
                            <div className="flex flex-col items-start">
                                <span className="font-semibold">
                                    {isAutoLanding
                                        ? "Auto-Landing..."
                                        : "Auto Landing - CV"}
                                </span>
                                <span className="text-xs opacity-80">
                                    {arUcoDetected
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
                        disabled={!isConnected || !isArmed}
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

                {(isTakingOff || isLanding || isAutoLanding) && (
                    <div className="p-3 bg-amber-950 border border-amber-800 rounded-lg">
                        <p className="text-sm text-amber-200 font-medium">
                            ⚠️ Operation in progress:{" "}
                            <span className="uppercase font-bold">
                                {isTakingOff
                                    ? "Takeoff"
                                    : isAutoLanding
                                    ? "Auto-Landing"
                                    : "Landing"}
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
