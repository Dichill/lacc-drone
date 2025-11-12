"use client";

import React, { useEffect, useRef } from "react";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Video, VideoOff } from "lucide-react";
import type { VideoFrame } from "@/hooks/use-mqtt";

interface VideoStreamProps {
    videoFrame: VideoFrame | null;
    isConnected: boolean;
}

export function VideoStream({ videoFrame, isConnected }: VideoStreamProps) {
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const frameCountRef = useRef<number>(0);
    const lastFrameTimeRef = useRef<number>(0);
    const fpsRef = useRef<number>(0);

    useEffect(() => {
        if (!videoFrame || !canvasRef.current) return;

        const canvas = canvasRef.current;
        const ctx = canvas.getContext("2d");
        if (!ctx) return;

        const img = new Image();

        img.onload = () => {
            canvas.width = img.width;
            canvas.height = img.height;

            ctx.drawImage(img, 0, 0);
            frameCountRef.current += 1;
            const now = Date.now();
            const elapsed = now - lastFrameTimeRef.current;

            if (elapsed >= 1000) {
                fpsRef.current = Math.round(
                    (frameCountRef.current * 1000) / elapsed
                );
                frameCountRef.current = 0;
                lastFrameTimeRef.current = now;
            }
        };

        if (videoFrame.data.startsWith("data:")) {
            img.src = videoFrame.data;
        } else {
            img.src = `data:image/jpeg;base64,${videoFrame.data}`;
        }
    }, [videoFrame]);

    return (
        <Card className="p-6 bg-slate-950 border-slate-800">
            <div className="space-y-4">
                <div className="flex items-center justify-between">
                    <div>
                        <h2 className="text-xl font-bold text-slate-200 uppercase tracking-wide">
                            Camera Feed
                        </h2>
                        <p className="text-sm text-slate-500 mt-1">
                            Live Video Stream
                        </p>
                    </div>
                    <div className="flex items-center gap-2">
                        <Badge
                            variant={videoFrame ? "default" : "secondary"}
                            className="uppercase"
                        >
                            {videoFrame ? (
                                <>
                                    <Video className="mr-2 h-3 w-3" />
                                    Streaming
                                </>
                            ) : (
                                <>
                                    <VideoOff className="mr-2 h-3 w-3" />
                                    No Signal
                                </>
                            )}
                        </Badge>
                        {videoFrame && fpsRef.current > 0 && (
                            <Badge
                                variant="outline"
                                className="font-mono text-xs"
                            >
                                {fpsRef.current} FPS
                            </Badge>
                        )}
                    </div>
                </div>

                <div className="relative bg-slate-900 rounded-lg border border-slate-800 overflow-hidden">
                    {!isConnected ? (
                        <div className="aspect-video flex items-center justify-center">
                            <div className="text-center text-slate-500">
                                <VideoOff className="h-16 w-16 mx-auto mb-4 opacity-50" />
                                <p className="text-sm">Not Connected</p>
                                <p className="text-xs mt-1">
                                    Connect to MQTT broker to view stream
                                </p>
                            </div>
                        </div>
                    ) : !videoFrame ? (
                        <div className="aspect-video flex items-center justify-center">
                            <div className="text-center text-slate-500">
                                <Video className="h-16 w-16 mx-auto mb-4 opacity-50 animate-pulse" />
                                <p className="text-sm">
                                    Waiting for Video Stream...
                                </p>
                                <p className="text-xs mt-1">
                                    Listening on topic: camera/stream
                                </p>
                            </div>
                        </div>
                    ) : (
                        <canvas
                            ref={canvasRef}
                            className="w-full h-auto"
                            style={{ maxHeight: "600px", objectFit: "contain" }}
                        />
                    )}
                </div>

                {videoFrame && (
                    <div className="flex items-center justify-between text-xs text-slate-500 font-mono">
                        <span>
                            Last frame:{" "}
                            {new Date(
                                videoFrame.timestamp
                            ).toLocaleTimeString()}
                        </span>
                        <span>Frames: {frameCountRef.current}</span>
                    </div>
                )}
            </div>
        </Card>
    );
}
