"use client";

import React from "react";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Scan, ScanSearch, AlertCircle } from "lucide-react";
import type { ArUcoDetection } from "@/hooks/use-mqtt";

interface ArUcoDetectionProps {
    detection: ArUcoDetection | null;
    isConnected: boolean;
}

export function ArUcoDetectionDisplay({
    detection,
    isConnected,
}: ArUcoDetectionProps) {
    const isDetected = detection?.detected ?? false;

    return (
        <Card className="p-6 bg-slate-950 border-slate-800">
            <div className="space-y-4">
                <div className="flex items-center justify-between">
                    <div>
                        <h2 className="text-xl font-bold text-slate-200 uppercase tracking-wide">
                            ArUco Detection
                        </h2>
                        <p className="text-sm text-slate-500 mt-1">
                            Computer Vision Marker Tracking
                        </p>
                    </div>
                    <Badge
                        variant={isDetected ? "default" : "secondary"}
                        className="uppercase"
                    >
                        {isDetected ? (
                            <>
                                <Scan className="mr-2 h-3 w-3" />
                                Detected
                            </>
                        ) : (
                            <>
                                <ScanSearch className="mr-2 h-3 w-3" />
                                Scanning
                            </>
                        )}
                    </Badge>
                </div>

                {!isConnected ? (
                    <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                        <div className="flex items-center gap-3 text-slate-500">
                            <AlertCircle className="h-8 w-8" />
                            <div>
                                <p className="text-sm font-medium">
                                    Not Connected
                                </p>
                                <p className="text-xs mt-1">
                                    Connect to MQTT broker to receive detection
                                    data
                                </p>
                            </div>
                        </div>
                    </div>
                ) : !detection ? (
                    <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                        <div className="flex items-center gap-3 text-slate-500">
                            <ScanSearch className="h-8 w-8 animate-pulse" />
                            <div>
                                <p className="text-sm font-medium">
                                    Waiting for Data...
                                </p>
                                <p className="text-xs mt-1">
                                    Listening on topic: drone/aruco_detection
                                </p>
                            </div>
                        </div>
                    </div>
                ) : isDetected ? (
                    <div className="space-y-3">
                        <div className="p-4 bg-emerald-950 border border-emerald-800 rounded-lg">
                            <div className="flex items-center gap-3 text-emerald-200">
                                <Scan className="h-8 w-8" />
                                <div>
                                    <p className="text-sm font-semibold">
                                        Marker Detected
                                    </p>
                                    <p className="text-xs mt-1 opacity-80">
                                        Auto-land available
                                    </p>
                                </div>
                            </div>
                        </div>

                        <div className="grid grid-cols-1 gap-3">
                            {detection.marker_id !== undefined && (
                                <div className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                                    <div className="flex items-center justify-between">
                                        <span className="text-xs text-slate-400 uppercase tracking-wide">
                                            Marker ID
                                        </span>
                                        <span className="text-2xl font-bold text-emerald-400 font-mono">
                                            {detection.marker_id}
                                        </span>
                                    </div>
                                </div>
                            )}

                            {detection.distance !== undefined && (
                                <div className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                                    <div className="flex items-center justify-between">
                                        <span className="text-xs text-slate-400 uppercase tracking-wide">
                                            Distance
                                        </span>
                                        <div className="text-right">
                                            <span className="text-2xl font-bold text-sky-400 font-mono">
                                                {detection.distance.toFixed(2)}
                                            </span>
                                            <span className="text-sm text-slate-400 ml-1">
                                                m
                                            </span>
                                        </div>
                                    </div>
                                    <div className="mt-2 w-full bg-slate-800 rounded-full h-2">
                                        <div
                                            className="bg-sky-500 h-2 rounded-full transition-all"
                                            style={{
                                                width: `${Math.min(
                                                    (10 /
                                                        (detection.distance ||
                                                            10)) *
                                                        100,
                                                    100
                                                )}%`,
                                            }}
                                        />
                                    </div>
                                </div>
                            )}

                            {detection.angle !== undefined && (
                                <div className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                                    <div className="flex items-center justify-between">
                                        <span className="text-xs text-slate-400 uppercase tracking-wide">
                                            Angle
                                        </span>
                                        <div className="text-right">
                                            <span className="text-2xl font-bold text-purple-400 font-mono">
                                                {detection.angle.toFixed(1)}
                                            </span>
                                            <span className="text-sm text-slate-400 ml-1">
                                                °
                                            </span>
                                        </div>
                                    </div>
                                    <div className="mt-2 flex items-center justify-center">
                                        <div
                                            className="w-16 h-16 border-2 border-purple-500 rounded-full relative"
                                            style={{
                                                transform: `rotate(${detection.angle}deg)`,
                                            }}
                                        >
                                            <div className="absolute top-0 left-1/2 w-0.5 h-8 bg-purple-400 -translate-x-1/2" />
                                        </div>
                                    </div>
                                </div>
                            )}
                        </div>

                        {/* Additional detection data */}
                        {Object.keys(detection).length > 4 && (
                            <details className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                                <summary className="text-xs text-slate-400 uppercase tracking-wide cursor-pointer">
                                    Additional Data
                                </summary>
                                <pre className="mt-2 text-xs text-slate-300 font-mono whitespace-pre-wrap break-words">
                                    {JSON.stringify(
                                        Object.fromEntries(
                                            Object.entries(detection).filter(
                                                ([key]) =>
                                                    ![
                                                        "detected",
                                                        "marker_id",
                                                        "distance",
                                                        "angle",
                                                    ].includes(key)
                                            )
                                        ),
                                        null,
                                        2
                                    )}
                                </pre>
                            </details>
                        )}
                    </div>
                ) : (
                    <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                        <div className="flex items-center gap-3 text-slate-500">
                            <ScanSearch className="h-8 w-8" />
                            <div>
                                <p className="text-sm font-medium">
                                    No Marker Detected
                                </p>
                                <p className="text-xs mt-1">
                                    Position camera to view ArUco marker
                                </p>
                            </div>
                        </div>
                    </div>
                )}
            </div>
        </Card>
    );
}
