"use client";

import React, { useState } from "react";
import { CircularGauge } from "@/components/gauges/circular-gauge";
import { LinearGauge } from "@/components/gauges/linear-gauge";
import { HeadingGauge } from "@/components/gauges/heading-gauge";
import { ControlPanel } from "@/components/control-panel";
import { ManualControls } from "@/components/manual-controls";
import { MQTTConnection } from "@/components/mqtt-connection";
import { VideoStream } from "@/components/video-stream";
import { ArUcoDetectionDisplay } from "@/components/aruco-detection";
import { Badge } from "@/components/ui/badge";
import { Activity, Wifi } from "lucide-react";
import { useMQTT } from "@/hooks/use-mqtt";

export default function Home() {
    const [lastUpdate, setLastUpdate] = useState<Date | null>(new Date());
    const [markerLocked, setMarkerLocked] = useState(false);
    const [lockedMarkerId, setLockedMarkerId] = useState<number | undefined>();

    const mqtt = useMQTT();

    React.useEffect(() => {
        if (mqtt.messageCount > 0) {
            setLastUpdate(new Date());
        }
    }, [mqtt.messageCount]);

    const handleMarkerLockChange = (locked: boolean, markerId?: number) => {
        setMarkerLocked(locked);
        setLockedMarkerId(markerId);
    };

    const telemetry = {
        altitude: mqtt.telemetry?.altitude ?? 0,
        ground_speed: mqtt.telemetry?.ground_speed ?? 0,
        vertical_speed: mqtt.telemetry?.vertical_speed ?? 0,
        heading: mqtt.telemetry?.heading ?? 0,
        armed: mqtt.telemetry?.armed ?? false,
        mode: mqtt.telemetry?.mode ?? "UNKNOWN",
        battery: mqtt.telemetry?.battery ?? null,
        gps_fix: mqtt.telemetry?.gps_fix ?? null,
        landing_mode: mqtt.telemetry?.landing_mode ?? false,
        centering_mode: mqtt.telemetry?.centering_mode ?? false,
        airspeed: mqtt.telemetry?.airspeed ?? 0,
        latitude: mqtt.telemetry?.latitude,
        longitude: mqtt.telemetry?.longitude,
        timestamp: mqtt.telemetry?.timestamp ?? 0,
    };

    const handleManualControl = (direction: string, active: boolean) => {
        console.log(
            `Manual control: ${direction} - ${active ? "ACTIVE" : "RELEASED"}`
        );
        // TODO: Implement manual control via MQTT if needed
    };

    return (
        <div className="min-h-screen bg-gradient-to-br from-slate-950 via-slate-900 to-slate-950 p-4 md:p-6 lg:p-8">
            <header className="mb-6">
                <div className="flex items-start justify-between flex-wrap gap-4">
                    <div>
                        <h1 className="text-3xl md:text-4xl font-bold text-slate-100 uppercase tracking-tight">
                            Ground Station
                        </h1>
                        <p className="text-slate-400 mt-1 text-sm md:text-base">
                            Los Angeles City College - Drone Operations Center
                        </p>
                    </div>
                    <div className="flex items-center gap-3">
                        <Badge
                            variant={
                                mqtt.isConnected ? "default" : "destructive"
                            }
                            className="px-3 py-1 text-sm"
                        >
                            <Wifi className="mr-2 h-4 w-4" />
                            {mqtt.isConnected ? "Connected" : "Disconnected"}
                        </Badge>
                        <Badge
                            variant="outline"
                            className="px-3 py-1 text-sm border-slate-700"
                        >
                            <Activity className="mr-2 h-4 w-4" />
                            {mqtt.messageCount > 0 ? "Live" : "Idle"}
                        </Badge>
                    </div>
                </div>
                <div className="mt-4 text-xs text-slate-500 font-mono">
                    Last Update: {lastUpdate?.toLocaleTimeString() || "—"} |
                    Messages: {mqtt.messageCount}
                </div>
            </header>

            <div className="grid grid-cols-1 xl:grid-cols-3 gap-6">
                {/* Left Column - Video Feed & ArUco Detection */}
                <div className="xl:col-span-2 space-y-6">
                    {/* Video Stream */}
                    <div>
                        <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                            <div className="w-1 h-5 bg-purple-500 mr-3 rounded" />
                            Camera Feed
                        </h2>
                        <VideoStream
                            videoFrame={mqtt.videoFrame}
                            isConnected={mqtt.isConnected}
                        />
                    </div>

                    {/* ArUco Detection */}
                    <div>
                        <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                            <div className="w-1 h-5 bg-emerald-500 mr-3 rounded" />
                            Marker Detection
                        </h2>
                        <ArUcoDetectionDisplay
                            detection={mqtt.arUcoDetection}
                            isConnected={mqtt.isConnected}
                            onLockChange={handleMarkerLockChange}
                        />
                    </div>

                    {/* Flight Instruments */}
                    <div>
                        <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                            <div className="w-1 h-5 bg-sky-500 mr-3 rounded" />
                            Primary Flight Instruments
                        </h2>
                        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                            <CircularGauge
                                value={telemetry.altitude}
                                min={0}
                                max={150}
                                label="Altitude"
                                unit="meters"
                                warningThreshold={120}
                                dangerThreshold={140}
                                dangerOnHigh={true}
                            />
                            <CircularGauge
                                value={telemetry.ground_speed}
                                min={0}
                                max={25}
                                label="Ground Speed"
                                unit="m/s"
                                warningThreshold={20}
                                dangerThreshold={23}
                                dangerOnHigh={true}
                            />
                            <CircularGauge
                                value={telemetry.airspeed}
                                min={0}
                                max={25}
                                label="Airspeed"
                                unit="m/s"
                                warningThreshold={20}
                                dangerThreshold={23}
                                dangerOnHigh={true}
                            />
                        </div>
                    </div>

                    {/* Navigation & Orientation */}
                    <div>
                        <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                            <div className="w-1 h-5 bg-purple-500 mr-3 rounded" />
                            Navigation &amp; Orientation
                        </h2>
                        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                            <HeadingGauge
                                yaw={telemetry.heading}
                                label="Yaw / Heading"
                            />
                            <div className="h-[290px]">
                                <LinearGauge
                                    value={telemetry.vertical_speed}
                                    min={-5}
                                    max={5}
                                    label="Vertical Speed"
                                    unit="m/s"
                                />
                            </div>
                        </div>
                    </div>

                    {mqtt.telemetry &&
                        telemetry.latitude !== undefined &&
                        telemetry.longitude !== undefined && (
                            <div>
                                <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                                    <div className="w-1 h-5 bg-emerald-500 mr-3 rounded" />
                                    GPS Position
                                </h2>
                                <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                                    <div className="grid grid-cols-2 gap-4 text-sm">
                                        <div>
                                            <span className="text-slate-500">
                                                Latitude:
                                            </span>
                                            <div className="mt-1 text-slate-200 font-mono text-lg">
                                                {telemetry.latitude.toFixed(6)}°
                                            </div>
                                        </div>
                                        <div>
                                            <span className="text-slate-500">
                                                Longitude:
                                            </span>
                                            <div className="mt-1 text-slate-200 font-mono text-lg">
                                                {telemetry.longitude.toFixed(6)}
                                                °
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        )}
                </div>

                {/* Right Column - Controls */}
                <div className="xl:col-span-1 space-y-6">
                    {/* MQTT Connection */}
                    <div>
                        <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                            <div className="w-1 h-5 bg-purple-500 mr-3 rounded" />
                            Connection
                        </h2>
                        <MQTTConnection mqtt={mqtt} />
                    </div>

                    {/* Mission Control */}
                    <div>
                        <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                            <div className="w-1 h-5 bg-emerald-500 mr-3 rounded" />
                            Mission Control
                        </h2>
                        <ControlPanel
                            mqtt={mqtt}
                            arUcoDetection={mqtt.arUcoDetection}
                            markerLocked={markerLocked}
                            lockedMarkerId={lockedMarkerId}
                        />
                    </div>

                    {/* Manual Flight Control */}
                    <div>
                        <h2 className="text-lg font-semibold text-slate-300 uppercase tracking-wide mb-4 flex items-center">
                            <div className="w-1 h-5 bg-sky-500 mr-3 rounded" />
                            Manual Flight Control
                        </h2>
                        <ManualControls
                            onControl={handleManualControl}
                            enabled={mqtt.isConnected}
                        />
                    </div>
                </div>
            </div>

            <footer className="mt-8 pt-6 border-t border-slate-800">
                <div className="text-center text-xs text-slate-600">
                    <p>Los Angeles City College - Developed by SHPE</p>
                    <p className="mt-1">YEAR 2024-2025</p>
                    <p className="mt-2 text-slate-700">
                        MQTT Protocol | Topics: drone/commands, camera/stream,
                        drone/aruco_detection
                    </p>
                </div>
            </footer>
        </div>
    );
}
