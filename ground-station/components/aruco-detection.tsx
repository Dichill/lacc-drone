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
const LIVE_MARKER_TIMEOUT = 2000;
const CLEANUP_INTERVAL = 500;

export function ArUcoDetectionDisplay({
    detection,
    isConnected,
    onLockChange,
    autoLockEnabled = false,
}: ArUcoDetectionProps) {
    const [isLocked, setIsLocked] = useState(false);
    const [lockedMarkerId, setLockedMarkerId] = useState<number | null>(null);
    const [cachedMarkers, setCachedMarkers] = useState<
        Map<number, CachedMarker>
    >(new Map());
    const [hasEverDetected, setHasEverDetected] = useState(false);
    const [hasAdditionalData, setHasAdditionalData] = useState(false);

    const hasActiveMarkers = cachedMarkers.size > 0;
    const isDetected = detection?.detected ?? false;

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
            const now = Date.now();

            setHasEverDetected(true);

            if (!hasAdditionalData && detection) {
                const additionalKeys = Object.keys(detection).filter(
                    (key) =>
                        ![
                            "detected",
                            "marker_id",
                            "distance",
                            "angle",
                        ].includes(key)
                );
                if (additionalKeys.length > 0) {
                    setHasAdditionalData(true);
                }
            }

            setCachedMarkers((prev) => {
                const updated = new Map(prev);
                const existing = prev.get(markerId);

                if (!existing || now - existing.lastSeen > 100) {
                    updated.set(markerId, {
                        id: markerId,
                        lastSeen: now,
                        isLive: true,
                    });
                    return updated;
                }

                return prev;
            });

            if (autoLockEnabled && !isLocked) {
                handleLockIn(markerId);
            }
        }
    }, [
        isDetected,
        detection,
        detection?.marker_id,
        autoLockEnabled,
        isLocked,
        hasAdditionalData,
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

                        if (isLocked && id === lockedMarkerId) {
                            handleUnlock();
                        }
                    } else if (
                        marker.isLive &&
                        timeSinceLastSeen > LIVE_MARKER_TIMEOUT
                    ) {
                        updated.set(id, { ...marker, isLive: false });
                        hasChanges = true;
                    }
                });

                return hasChanges ? updated : prev;
            });
        }, CLEANUP_INTERVAL);

        return () => clearInterval(interval);
    }, [isLocked, lockedMarkerId, handleUnlock]);

    const cachedMarkerArray = Array.from(cachedMarkers.values()).sort(
        (a, b) => b.lastSeen - a.lastSeen
    );

    const lockedMarkerInCache =
        isLocked && lockedMarkerId !== null
            ? cachedMarkers.get(lockedMarkerId)
            : null;

    const isCorrectMarker = isLocked && lockedMarkerInCache !== undefined;
    const isLockedMarkerLive = lockedMarkerInCache?.isLive ?? false;

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
                                    : hasActiveMarkers
                                    ? "default"
                                    : "secondary"
                            }
                            className={
                                isLocked && isCorrectMarker
                                    ? "uppercase bg-emerald-600"
                                    : "uppercase"
                            }
                        >
                            {hasActiveMarkers ? (
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
                ) : hasEverDetected || hasActiveMarkers ? (
                    <div className="space-y-3">
                        <div className="p-4 bg-slate-900 rounded-lg border border-slate-800 min-h-[120px]">
                            <h4 className="text-sm font-semibold text-slate-300 mb-3">
                                Detected Markers
                            </h4>
                            {hasActiveMarkers ? (
                                <div className="space-y-2">
                                    {cachedMarkerArray.map((marker) => (
                                        <div
                                            key={marker.id}
                                            className={`p-3 rounded-lg border flex items-center justify-between transition-all duration-200 ${
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
                            ) : (
                                <div className="flex items-center gap-3 text-slate-500 py-4">
                                    <ScanSearch className="h-6 w-6 animate-pulse" />
                                    <div>
                                        <p className="text-sm font-medium">
                                            Scanning for markers...
                                        </p>
                                        <p className="text-xs mt-1 opacity-80">
                                            No markers detected in the last 5
                                            seconds
                                        </p>
                                    </div>
                                </div>
                            )}
                        </div>

                        <div
                            className={`p-4 rounded-lg transition-colors duration-300 ${
                                isCorrectMarker
                                    ? "bg-emerald-950 border border-emerald-800"
                                    : isLocked
                                    ? "bg-red-950 border border-red-800"
                                    : "bg-emerald-950 border border-emerald-800"
                            }`}
                        >
                            <div
                                className={`flex items-center gap-3 ${
                                    isCorrectMarker
                                        ? "text-emerald-200"
                                        : isLocked
                                        ? "text-red-200"
                                        : "text-emerald-200"
                                }`}
                            >
                                {isCorrectMarker ? (
                                    isLockedMarkerLive ? (
                                        <Scan className="h-8 w-8" />
                                    ) : (
                                        <Clock className="h-8 w-8" />
                                    )
                                ) : (
                                    <ScanSearch className="h-8 w-8 animate-pulse" />
                                )}
                                <div className="flex-1">
                                    <p className="text-sm font-semibold">
                                        {isCorrectMarker
                                            ? isLockedMarkerLive
                                                ? `Locked on Marker ${lockedMarkerId} - Live`
                                                : `Locked on Marker ${lockedMarkerId} - Cached`
                                            : isLocked
                                            ? `Searching for Marker ${lockedMarkerId}`
                                            : "Marker Detected"}
                                    </p>
                                    <p className="text-xs mt-1 opacity-80">
                                        {isCorrectMarker
                                            ? "Ready for auto-land"
                                            : isLocked
                                            ? "Marker lost - expires in 5s"
                                            : "Lock marker to enable auto-land"}
                                    </p>
                                </div>
                                {isLocked && (
                                    <Button
                                        onClick={handleUnlock}
                                        variant="outline"
                                        size="sm"
                                        className={
                                            isCorrectMarker
                                                ? "border-emerald-700 text-emerald-200 hover:bg-emerald-900"
                                                : "border-red-700 text-red-200 hover:bg-red-900"
                                        }
                                    >
                                        <Unlock className="mr-2 h-3 w-3" />
                                        Unlock
                                    </Button>
                                )}
                            </div>
                        </div>

                        {/* Additional detection data */}
                        {hasAdditionalData && detection && (
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
                    <div className="p-4 bg-slate-900 rounded-lg border border-slate-800 min-h-[120px] flex items-center">
                        <div className="flex items-center gap-3 text-slate-500">
                            <ScanSearch className="h-8 w-8 animate-pulse" />
                            <div>
                                <p className="text-sm font-medium">
                                    Scanning for markers...
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
