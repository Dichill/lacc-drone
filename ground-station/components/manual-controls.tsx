"use client";

import { useState } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import {
    ArrowUp,
    ArrowDown,
    ArrowLeft,
    ArrowRight,
    ChevronUp,
    ChevronDown,
    RotateCw,
    RotateCcw,
} from "lucide-react";

type Direction =
    | "forward"
    | "backward"
    | "left"
    | "right"
    | "up"
    | "down"
    | "yaw-left"
    | "yaw-right";

interface ManualControlsProps {
    onControl?: (direction: Direction, active: boolean) => void;
    enabled?: boolean;
}

export function ManualControls({
    onControl,
    enabled = true,
}: ManualControlsProps) {
    const [activeControls, setActiveControls] = useState<Set<Direction>>(
        new Set()
    );
    const [throttle, setThrottle] = useState(50);

    /**
     * Handle control button press
     * @param direction - The direction to move
     */
    const handleControlPress = (direction: Direction) => {
        if (!enabled) return;

        const newActive = new Set(activeControls);
        newActive.add(direction);
        setActiveControls(newActive);
        onControl?.(direction, true);
    };

    /**
     * Handle control button release
     * @param direction - The direction to stop
     */
    const handleControlRelease = (direction: Direction) => {
        if (!enabled) return;

        const newActive = new Set(activeControls);
        newActive.delete(direction);
        setActiveControls(newActive);
        onControl?.(direction, false);
    };

    /**
     * Check if a control is active
     * @param direction - The direction to check
     * @returns Whether the control is active
     */
    const isActive = (direction: Direction): boolean => {
        return activeControls.has(direction);
    };

    return (
        <Card className="p-6 bg-slate-950 border-slate-800">
            <div className="space-y-6">
                <div className="flex items-center justify-between">
                    <div>
                        <h2 className="text-xl font-bold text-slate-200 uppercase tracking-wide">
                            Manual Controls
                        </h2>
                        <p className="text-sm text-slate-500 mt-1">
                            Direct flight control
                        </p>
                    </div>
                    <Badge
                        variant={enabled ? "default" : "secondary"}
                        className="uppercase"
                    >
                        {enabled ? "Active" : "Disabled"}
                    </Badge>
                </div>

                <div className="grid grid-cols-2 gap-6">
                    <div className="space-y-4">
                        <h3 className="text-sm font-semibold text-slate-400 uppercase tracking-wide text-center">
                            Throttle &amp; Yaw
                        </h3>

                        <div className="flex flex-col items-center bg-slate-900 rounded-lg border border-slate-800 p-4">
                            <div className="text-xs text-slate-500 uppercase tracking-wide mb-2">
                                Throttle
                            </div>
                            <div className="flex flex-col items-center gap-2 mb-3">
                                <Button
                                    size="lg"
                                    disabled={!enabled}
                                    onMouseDown={() => handleControlPress("up")}
                                    onMouseUp={() => handleControlRelease("up")}
                                    onMouseLeave={() =>
                                        handleControlRelease("up")
                                    }
                                    onTouchStart={() =>
                                        handleControlPress("up")
                                    }
                                    onTouchEnd={() =>
                                        handleControlRelease("up")
                                    }
                                    className={`w-16 h-16 rounded-lg ${
                                        isActive("up")
                                            ? "bg-emerald-600 hover:bg-emerald-700 glow-emerald"
                                            : "bg-slate-800 hover:bg-slate-700"
                                    }`}
                                >
                                    <ChevronUp className="h-8 w-8" />
                                </Button>

                                <div className="relative w-12 h-32 bg-slate-950 rounded border border-slate-700">
                                    <div
                                        className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-emerald-600 to-emerald-400 rounded transition-all"
                                        style={{ height: `${throttle}%` }}
                                    />
                                    <div className="absolute inset-0 flex items-center justify-center">
                                        <span className="text-xs font-bold text-slate-300 z-10 drop-shadow-lg">
                                            {throttle}%
                                        </span>
                                    </div>
                                </div>

                                <Button
                                    size="lg"
                                    disabled={!enabled}
                                    onMouseDown={() =>
                                        handleControlPress("down")
                                    }
                                    onMouseUp={() =>
                                        handleControlRelease("down")
                                    }
                                    onMouseLeave={() =>
                                        handleControlRelease("down")
                                    }
                                    onTouchStart={() =>
                                        handleControlPress("down")
                                    }
                                    onTouchEnd={() =>
                                        handleControlRelease("down")
                                    }
                                    className={`w-16 h-16 rounded-lg ${
                                        isActive("down")
                                            ? "bg-orange-600 hover:bg-orange-700 glow-amber"
                                            : "bg-slate-800 hover:bg-slate-700"
                                    }`}
                                >
                                    <ChevronDown className="h-8 w-8" />
                                </Button>
                            </div>
                        </div>

                        <div className="flex justify-center gap-2 bg-slate-900 rounded-lg border border-slate-800 p-4">
                            <div className="flex flex-col items-center w-full">
                                <div className="text-xs text-slate-500 uppercase tracking-wide mb-2">
                                    Yaw
                                </div>
                                <div className="flex gap-2">
                                    <Button
                                        size="lg"
                                        disabled={!enabled}
                                        onMouseDown={() =>
                                            handleControlPress("yaw-left")
                                        }
                                        onMouseUp={() =>
                                            handleControlRelease("yaw-left")
                                        }
                                        onMouseLeave={() =>
                                            handleControlRelease("yaw-left")
                                        }
                                        onTouchStart={() =>
                                            handleControlPress("yaw-left")
                                        }
                                        onTouchEnd={() =>
                                            handleControlRelease("yaw-left")
                                        }
                                        className={`w-16 h-16 rounded-lg ${
                                            isActive("yaw-left")
                                                ? "bg-purple-600 hover:bg-purple-700 glow-sky"
                                                : "bg-slate-800 hover:bg-slate-700"
                                        }`}
                                    >
                                        <RotateCcw className="h-6 w-6" />
                                    </Button>
                                    <Button
                                        size="lg"
                                        disabled={!enabled}
                                        onMouseDown={() =>
                                            handleControlPress("yaw-right")
                                        }
                                        onMouseUp={() =>
                                            handleControlRelease("yaw-right")
                                        }
                                        onMouseLeave={() =>
                                            handleControlRelease("yaw-right")
                                        }
                                        onTouchStart={() =>
                                            handleControlPress("yaw-right")
                                        }
                                        onTouchEnd={() =>
                                            handleControlRelease("yaw-right")
                                        }
                                        className={`w-16 h-16 rounded-lg ${
                                            isActive("yaw-right")
                                                ? "bg-purple-600 hover:bg-purple-700 glow-sky"
                                                : "bg-slate-800 hover:bg-slate-700"
                                        }`}
                                    >
                                        <RotateCw className="h-6 w-6" />
                                    </Button>
                                </div>
                            </div>
                        </div>
                    </div>

                    <div className="space-y-4">
                        <h3 className="text-sm font-semibold text-slate-400 uppercase tracking-wide text-center">
                            Directional
                        </h3>

                        <div className="flex flex-col items-center bg-slate-900 rounded-lg border border-slate-800 p-4">
                            <div className="text-xs text-slate-500 uppercase tracking-wide mb-3">
                                Horizontal Movement
                            </div>

                            <div className="grid grid-cols-3 gap-2">
                                <div />
                                <Button
                                    size="lg"
                                    disabled={!enabled}
                                    onMouseDown={() =>
                                        handleControlPress("forward")
                                    }
                                    onMouseUp={() =>
                                        handleControlRelease("forward")
                                    }
                                    onMouseLeave={() =>
                                        handleControlRelease("forward")
                                    }
                                    onTouchStart={() =>
                                        handleControlPress("forward")
                                    }
                                    onTouchEnd={() =>
                                        handleControlRelease("forward")
                                    }
                                    className={`w-16 h-16 rounded-lg ${
                                        isActive("forward")
                                            ? "bg-sky-600 hover:bg-sky-700 glow-sky"
                                            : "bg-slate-800 hover:bg-slate-700"
                                    }`}
                                >
                                    <ArrowUp className="h-8 w-8" />
                                </Button>
                                <div />

                                <Button
                                    size="lg"
                                    disabled={!enabled}
                                    onMouseDown={() =>
                                        handleControlPress("left")
                                    }
                                    onMouseUp={() =>
                                        handleControlRelease("left")
                                    }
                                    onMouseLeave={() =>
                                        handleControlRelease("left")
                                    }
                                    onTouchStart={() =>
                                        handleControlPress("left")
                                    }
                                    onTouchEnd={() =>
                                        handleControlRelease("left")
                                    }
                                    className={`w-16 h-16 rounded-lg ${
                                        isActive("left")
                                            ? "bg-sky-600 hover:bg-sky-700 glow-sky"
                                            : "bg-slate-800 hover:bg-slate-700"
                                    }`}
                                >
                                    <ArrowLeft className="h-8 w-8" />
                                </Button>
                                <div className="w-16 h-16 flex items-center justify-center bg-slate-950 rounded-lg border border-slate-700">
                                    <div className="w-3 h-3 rounded-full bg-slate-600" />
                                </div>
                                <Button
                                    size="lg"
                                    disabled={!enabled}
                                    onMouseDown={() =>
                                        handleControlPress("right")
                                    }
                                    onMouseUp={() =>
                                        handleControlRelease("right")
                                    }
                                    onMouseLeave={() =>
                                        handleControlRelease("right")
                                    }
                                    onTouchStart={() =>
                                        handleControlPress("right")
                                    }
                                    onTouchEnd={() =>
                                        handleControlRelease("right")
                                    }
                                    className={`w-16 h-16 rounded-lg ${
                                        isActive("right")
                                            ? "bg-sky-600 hover:bg-sky-700 glow-sky"
                                            : "bg-slate-800 hover:bg-slate-700"
                                    }`}
                                >
                                    <ArrowRight className="h-8 w-8" />
                                </Button>

                                <div />
                                <Button
                                    size="lg"
                                    disabled={!enabled}
                                    onMouseDown={() =>
                                        handleControlPress("backward")
                                    }
                                    onMouseUp={() =>
                                        handleControlRelease("backward")
                                    }
                                    onMouseLeave={() =>
                                        handleControlRelease("backward")
                                    }
                                    onTouchStart={() =>
                                        handleControlPress("backward")
                                    }
                                    onTouchEnd={() =>
                                        handleControlRelease("backward")
                                    }
                                    className={`w-16 h-16 rounded-lg ${
                                        isActive("backward")
                                            ? "bg-sky-600 hover:bg-sky-700 glow-sky"
                                            : "bg-slate-800 hover:bg-slate-700"
                                    }`}
                                >
                                    <ArrowDown className="h-8 w-8" />
                                </Button>
                                <div />
                            </div>
                        </div>
                    </div>
                </div>

                {activeControls.size > 0 && (
                    <div className="p-3 bg-sky-950 border border-sky-800 rounded-lg">
                        <p className="text-sm text-sky-200 font-medium">
                            ✈️ Active:{" "}
                            {Array.from(activeControls)
                                .map((dir) =>
                                    dir.toUpperCase().replace("-", " ")
                                )
                                .join(", ")}
                        </p>
                    </div>
                )}
            </div>
        </Card>
    );
}
