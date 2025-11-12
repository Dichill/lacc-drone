"use client";

import React, { useState, useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import {
    Scan,
    ScanSearch,
    AlertCircle,
    Lock,
    Unlock,
    Clock,
} from "lucide-react";
import type { ArUcoDetection } from "@/hooks/use-mqtt";

interface CachedMarker {
    id: number;
    lastSeen: number;
    isLive: boolean;
}

interface ArUcoDetectionProps {
    detection: ArUcoDetection | null;
    isConnected: boolean;
    onLockChange?: (locked: boolean, markerId?: number) => void;
    autoLockEnabled?: boolean;
}

const MARKER_CACHE_TIMEOUT = 5000;

export function ArUcoDetectionDisplay({
    detection,
    isConnected,
    onLockChange,
    autoLockEnabled = false,
}: ArUcoDetectionProps) {
    const isDetected = detection?.detected ?? false;
    const [isLocked, setIsLocked] = useState(false);
    const [lockedMarkerId, setLockedMarkerId] = useState<number | null>(null);
    const [cachedMarkers, setCachedMarkers] = useState<
        Map<number, CachedMarker>
    >(new Map());

    const handleLockIn = React.useCallback(
        (markerId: number) => {
            setIsLocked(true);
            setLockedMarkerId(markerId);
            onLockChange?.(true, markerId);
        },
        [onLockChange]
    );

    const handleUnlock = React.useCallback(() => {
        setIsLocked(false);
        setLockedMarkerId(null);
        onLockChange?.(false);
    }, [onLockChange]);

    useEffect(() => {
        if (isDetected && detection?.marker_id !== undefined) {
            const markerId = detection.marker_id;
            setCachedMarkers((prev) => {
                const updated = new Map(prev);
                updated.set(markerId, {
                    id: markerId,
                    lastSeen: Date.now(),
                    isLive: true,
                });
                return updated;
            });

            if (autoLockEnabled && !isLocked) {
                handleLockIn(markerId);
            }
        }
    }, [
        isDetected,
        detection?.marker_id,
        autoLockEnabled,
        isLocked,
        handleLockIn,
    ]);

    useEffect(() => {
        const interval = setInterval(() => {
            const now = Date.now();
            setCachedMarkers((prev) => {
                const updated = new Map(prev);
                let hasChanges = false;

                prev.forEach((marker, id) => {
                    const timeSinceLastSeen = now - marker.lastSeen;

                    if (timeSinceLastSeen > MARKER_CACHE_TIMEOUT) {
                        updated.delete(id);
                        hasChanges = true;
                    } else if (marker.isLive && timeSinceLastSeen > 1000) {
                        updated.set(id, { ...marker, isLive: false });
                        hasChanges = true;
                    }
                });

                return hasChanges ? updated : prev;
            });
        }, 250);

        return () => clearInterval(interval);
    }, []);

    const isCorrectMarker = isLocked && detection?.marker_id === lockedMarkerId;
    const cachedMarkerArray = Array.from(cachedMarkers.values()).sort(
        (a, b) => b.lastSeen - a.lastSeen
    );

    const getTimeSinceLastSeen = (lastSeen: number) => {
        const seconds = Math.floor((Date.now() - lastSeen) / 1000);
        return seconds === 0 ? "just now" : `${seconds}s ago`;
    };

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
                    <div className="flex gap-2">
                        {isLocked && (
                            <Badge
                                variant="default"
                                className="uppercase bg-amber-600"
                            >
                                <Lock className="mr-2 h-3 w-3" />
                                Locked
                            </Badge>
                        )}
                        <Badge
                            variant={
                                isLocked && isCorrectMarker
                                    ? "default"
                                    : isDetected
                                    ? "default"
                                    : "secondary"
                            }
                            className={
                                isLocked && isCorrectMarker
                                    ? "uppercase bg-emerald-600"
                                    : "uppercase"
                            }
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
                ) : cachedMarkerArray.length > 0 || isDetected ? (
                    <div className="space-y-3">
                        <div className="p-4 bg-slate-900 rounded-lg border border-slate-800">
                            <h4 className="text-sm font-semibold text-slate-300 mb-3">
                                Detected Markers
                            </h4>
                            <div className="space-y-2">
                                {cachedMarkerArray.map((marker) => (
                                    <div
                                        key={marker.id}
                                        className={`p-3 rounded-lg border flex items-center justify-between ${
                                            isLocked &&
                                            lockedMarkerId === marker.id
                                                ? "bg-emerald-950 border-emerald-800"
                                                : "bg-slate-800 border-slate-700"
                                        }`}
                                    >
                                        <div className="flex-1">
                                            <div className="flex items-center gap-2">
                                                <span className="text-lg font-bold text-slate-200 font-mono">
                                                    ID {marker.id}
                                                </span>
                                                {marker.isLive ? (
                                                    <Badge
                                                        variant="default"
                                                        className="text-xs bg-emerald-600"
                                                    >
                                                        <Scan className="mr-1 h-3 w-3" />
                                                        Live
                                                    </Badge>
                                                ) : (
                                                    <Badge
                                                        variant="secondary"
                                                        className="text-xs"
                                                    >
                                                        <Clock className="mr-1 h-3 w-3" />
                                                        {getTimeSinceLastSeen(
                                                            marker.lastSeen
                                                        )}
                                                    </Badge>
                                                )}
                                            </div>
                                        </div>
                                        {isLocked &&
                                        lockedMarkerId === marker.id ? (
                                            <Button
                                                onClick={handleUnlock}
                                                variant="outline"
                                                size="sm"
                                                className="border-emerald-700 text-emerald-200 hover:bg-emerald-900"
                                            >
                                                <Unlock className="mr-2 h-3 w-3" />
                                                Unlock
                                            </Button>
                                        ) : !isLocked ? (
                                            <Button
                                                onClick={() =>
                                                    handleLockIn(marker.id)
                                                }
                                                size="sm"
                                                className="bg-purple-600 hover:bg-purple-700"
                                            >
                                                <Lock className="mr-2 h-3 w-3" />
                                                Lock
                                            </Button>
                                        ) : null}
                                    </div>
                                ))}
                            </div>
                        </div>

                        <div
                            className={`p-4 rounded-lg ${
                                isLocked && isCorrectMarker
                                    ? "bg-emerald-950 border border-emerald-800"
                                    : isLocked && !isCorrectMarker
                                    ? "bg-red-950 border border-red-800"
                                    : "bg-emerald-950 border border-emerald-800"
                            }`}
                        >
                            <div
                                className={`flex items-center gap-3 ${
                                    isLocked && isCorrectMarker
                                        ? "text-emerald-200"
                                        : isLocked && !isCorrectMarker
                                        ? "text-red-200"
                                        : "text-emerald-200"
                                }`}
                            >
                                <Scan className="h-8 w-8" />
                                <div className="flex-1">
                                    <p className="text-sm font-semibold">
                                        {isLocked && isCorrectMarker
                                            ? "Locked Target Detected"
                                            : isLocked && !isCorrectMarker
                                            ? "Wrong Marker Detected"
                                            : "Marker Detected"}
                                    </p>
                                    <p className="text-xs mt-1 opacity-80">
                                        {isLocked && isCorrectMarker
                                            ? "Ready for auto-land"
                                            : isLocked && !isCorrectMarker
                                            ? `Looking for Marker ${lockedMarkerId}`
                                            : "Auto-land available after lock"}
                                    </p>
                                </div>
                                {isLocked && (
                                    <Button
                                        onClick={handleUnlock}
                                        variant="outline"
                                        size="sm"
                                        className="border-slate-700"
                                    >
                                        <Unlock className="mr-2 h-3 w-3" />
                                        Unlock
                                    </Button>
                                )}
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
