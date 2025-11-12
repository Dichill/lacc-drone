"use client";

import { Card } from "@/components/ui/card";

interface LinearGaugeProps {
    value: number;
    min: number;
    max: number;
    label: string;
    unit: string;
}

export function LinearGauge({
    value,
    min,
    max,
    label,
    unit,
}: LinearGaugeProps) {
    const normalizedValue = Math.max(min, Math.min(max, value));
    const percentage = ((normalizedValue - min) / (max - min)) * 100;

    const getColor = () => {
        if (value > 2) return "#22c55e";
        if (value < -2) return "#ef4444";
        if (value > 0.5) return "#84cc16";
        if (value < -0.5) return "#fb923c";
        return "#0ea5e9";
    };

    return (
        <Card className="p-4 bg-slate-950 border-slate-800 hover:border-sky-500/50 transition-colors h-full">
            <div className="flex flex-col items-center h-full">
                <div className="text-sm text-slate-500 uppercase tracking-wide font-semibold mb-3">
                    {label}
                </div>

                <div className="relative w-20 flex-1 bg-slate-900 rounded-lg border border-slate-700">
                    <div className="absolute left-0 right-0 top-1/2 h-[2px] bg-slate-600 z-10" />

                    {[-4, -2, 0, 2, 4].map((tick) => {
                        const tickPercentage =
                            ((tick - min) / (max - min)) * 100;
                        return (
                            <div
                                key={tick}
                                className="absolute left-0 right-0 flex items-center"
                                style={{ top: `${100 - tickPercentage}%` }}
                            >
                                <div className="w-2 h-[1px] bg-slate-600" />
                                <span className="text-[10px] text-slate-500 ml-1 font-mono">
                                    {tick}
                                </span>
                            </div>
                        );
                    })}

                    <div
                        className="absolute left-1 right-1 transition-all duration-300 rounded"
                        style={{
                            bottom: percentage < 50 ? `${percentage}%` : "50%",
                            top:
                                percentage > 50
                                    ? `${100 - percentage}%`
                                    : "50%",
                            backgroundColor: getColor(),
                            minHeight: "2px",
                        }}
                    />
                </div>

                <div className="text-center mt-3">
                    <div
                        className="text-2xl font-bold font-mono tracking-wider"
                        style={{ color: getColor() }}
                    >
                        {value > 0 ? "+" : ""}
                        {value.toFixed(2)}
                    </div>
                    <div className="text-xs text-slate-400 uppercase tracking-widest mt-1">
                        {unit}
                    </div>
                </div>
            </div>
        </Card>
    );
}
