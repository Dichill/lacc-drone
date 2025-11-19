"use client";

import { useEffect, useRef } from "react";
import { Card } from "@/components/ui/card";

interface HeadingGaugeProps {
    yaw: number;
    label: string;
}

export function HeadingGauge({ yaw, label }: HeadingGaugeProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const normalizedYaw = ((yaw % 360) + 360) % 360;

    useEffect(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext("2d");
        if (!ctx) return;

        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const radius = Math.min(centerX, centerY) - 20;

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        ctx.save();
        ctx.translate(centerX, centerY);
        ctx.rotate((-normalizedYaw * Math.PI) / 180);

        ctx.strokeStyle = "#334155";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(0, 0, radius, 0, Math.PI * 2);
        ctx.stroke();

        ctx.fillStyle = "#0f172a";
        ctx.beginPath();
        ctx.arc(0, 0, radius - 5, 0, Math.PI * 2);
        ctx.fill();

        const directions = [
            { angle: 0, label: "N", color: "#ef4444" },
            { angle: 90, label: "E", color: "#94a3b8" },
            { angle: 180, label: "S", color: "#94a3b8" },
            { angle: 270, label: "W", color: "#94a3b8" },
        ];

        directions.forEach(({ angle, label: dirLabel, color }) => {
            const rad = (angle * Math.PI) / 180;
            ctx.save();
            ctx.rotate(rad);

            ctx.strokeStyle = color;
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(0, -radius + 10);
            ctx.lineTo(0, -radius + 25);
            ctx.stroke();

            ctx.rotate(-rad);
            ctx.fillStyle = color;
            ctx.font = "bold 16px sans-serif";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText(
                dirLabel,
                Math.sin(rad) * (radius - 35),
                -Math.cos(rad) * (radius - 35)
            );

            ctx.restore();
        });

        for (let i = 0; i < 360; i += 30) {
            if (i % 90 !== 0) {
                const rad = (i * Math.PI) / 180;
                ctx.save();
                ctx.rotate(rad);

                ctx.strokeStyle = "#475569";
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(0, -radius + 10);
                ctx.lineTo(0, -radius + 20);
                ctx.stroke();

                ctx.rotate(-rad);
                ctx.fillStyle = "#64748b";
                ctx.font = "10px monospace";
                ctx.textAlign = "center";
                ctx.textBaseline = "middle";
                ctx.fillText(
                    i.toString(),
                    Math.sin(rad) * (radius - 35),
                    -Math.cos(rad) * (radius - 35)
                );

                ctx.restore();
            }
        }

        ctx.restore();

        ctx.strokeStyle = "#0ea5e9";
        ctx.fillStyle = "#0ea5e9";
        ctx.lineWidth = 3;

        ctx.beginPath();
        ctx.moveTo(centerX, centerY - radius + 8);
        ctx.lineTo(centerX - 8, centerY - radius + 18);
        ctx.lineTo(centerX + 8, centerY - radius + 18);
        ctx.closePath();
        ctx.fill();

        ctx.strokeStyle = "#0ea5e9";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY - 15);
        ctx.lineTo(centerX, centerY + 15);
        ctx.moveTo(centerX - 20, centerY);
        ctx.lineTo(centerX + 20, centerY);
        ctx.moveTo(centerX - 8, centerY + 12);
        ctx.lineTo(centerX + 8, centerY + 12);
        ctx.stroke();
    }, [normalizedYaw]);

    return (
        <Card className="p-4 bg-white dark:bg-slate-950 border-slate-300 dark:border-slate-800 hover:border-sky-500/50 transition-colors">
            <div className="flex flex-col items-center">
                <canvas
                    ref={canvasRef}
                    width={200}
                    height={200}
                    className="mb-2"
                />
                <div className="text-center">
                    <div className="text-3xl font-bold font-mono text-sky-400 tracking-wider">
                        {normalizedYaw.toFixed(1)}°
                    </div>
                    <div className="text-sm text-slate-700 dark:text-slate-500 uppercase tracking-wide mt-2 font-semibold">
                        {label}
                    </div>
                </div>
            </div>
        </Card>
    );
}
