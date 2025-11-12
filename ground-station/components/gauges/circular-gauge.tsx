"use client";

import { useEffect, useRef } from "react";
import { Card } from "@/components/ui/card";

/**
 * Props for the CircularGauge component
 */
interface CircularGaugeProps {
    value: number;
    min: number;
    max: number;
    label: string;
    unit: string;
    warningThreshold?: number;
    dangerThreshold?: number;
    dangerOnHigh?: boolean;
}

export function CircularGauge({
    value,
    min,
    max,
    label,
    unit,
    warningThreshold,
    dangerThreshold,
    dangerOnHigh = true,
}: CircularGaugeProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null);

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext("2d");
        if (!ctx) return;

        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const radius = Math.min(centerX, centerY) - 20;

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        const startAngle = -Math.PI * 0.75;
        const endAngle = Math.PI * 0.75;
        const angleRange = endAngle - startAngle;
        const normalizedValue = Math.max(min, Math.min(max, value));
        const valueAngle =
            startAngle + ((normalizedValue - min) / (max - min)) * angleRange;

        let needleColor = "#0ea5e9";
        if (warningThreshold !== undefined && dangerThreshold !== undefined) {
            if (dangerOnHigh) {
                if (normalizedValue >= dangerThreshold) {
                    needleColor = "#ef4444";
                } else if (normalizedValue >= warningThreshold) {
                    needleColor = "#f59e0b";
                }
            } else {
                if (normalizedValue <= dangerThreshold) {
                    needleColor = "#ef4444";
                } else if (normalizedValue <= warningThreshold) {
                    needleColor = "#f59e0b";
                }
            }
        }

        ctx.strokeStyle = "#334155";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, startAngle, endAngle);
        ctx.stroke();

        ctx.strokeStyle = "#1e293b";
        ctx.lineWidth = 12;
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius - 10, startAngle, endAngle);
        ctx.stroke();

        ctx.strokeStyle = needleColor;
        ctx.lineWidth = 12;
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius - 10, startAngle, valueAngle);
        ctx.stroke();

        const majorTicks = 5;
        for (let i = 0; i <= majorTicks; i++) {
            const angle = startAngle + (angleRange * i) / majorTicks;
            const tickStartRadius = radius - 25;
            const tickEndRadius = radius - 15;

            ctx.strokeStyle = "#64748b";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(
                centerX + Math.cos(angle) * tickStartRadius,
                centerY + Math.sin(angle) * tickStartRadius
            );
            ctx.lineTo(
                centerX + Math.cos(angle) * tickEndRadius,
                centerY + Math.sin(angle) * tickEndRadius
            );
            ctx.stroke();

            const tickValue = min + ((max - min) * i) / majorTicks;
            const labelRadius = radius - 40;
            ctx.fillStyle = "#94a3b8";
            ctx.font = "10px monospace";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText(
                tickValue.toFixed(0),
                centerX + Math.cos(angle) * labelRadius,
                centerY + Math.sin(angle) * labelRadius
            );
        }

        ctx.fillStyle = "#475569";
        ctx.beginPath();
        ctx.arc(centerX, centerY, 8, 0, Math.PI * 2);
        ctx.fill();

        ctx.strokeStyle = needleColor;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(
            centerX + Math.cos(valueAngle) * (radius - 20),
            centerY + Math.sin(valueAngle) * (radius - 20)
        );
        ctx.stroke();

        ctx.fillStyle = needleColor;
        ctx.beginPath();
        ctx.arc(
            centerX + Math.cos(valueAngle) * (radius - 20),
            centerY + Math.sin(valueAngle) * (radius - 20),
            4,
            0,
            Math.PI * 2
        );
        ctx.fill();
    }, [value, min, max, warningThreshold, dangerThreshold, dangerOnHigh]);

    return (
        <Card className="p-4 bg-slate-950 border-slate-800 hover:border-sky-500/50 transition-colors">
            <div className="flex flex-col items-center">
                <canvas
                    ref={canvasRef}
                    width={200}
                    height={200}
                    className="mb-2"
                />
                <div className="text-center">
                    <div className="text-3xl font-bold font-mono text-sky-400 tracking-wider">
                        {value.toFixed(1)}
                    </div>
                    <div className="text-xs text-slate-400 uppercase tracking-widest mt-1">
                        {unit}
                    </div>
                    <div className="text-sm text-slate-500 uppercase tracking-wide mt-2 font-semibold">
                        {label}
                    </div>
                </div>
            </div>
        </Card>
    );
}
