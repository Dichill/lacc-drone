"use client";

import React from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Wifi, WifiOff, Activity, Radio } from "lucide-react";
import type { UseMQTTReturn } from "@/hooks/use-mqtt";

interface MQTTConnectionProps {
    mqtt: UseMQTTReturn;
    onConnectionReady?: (mqttMethods: UseMQTTReturn) => void;
}

export function MQTTConnection({
    mqtt,
    onConnectionReady,
}: MQTTConnectionProps) {
    React.useEffect(() => {
        if (mqtt.isConnected && onConnectionReady) {
            onConnectionReady(mqtt);
        }
    }, [mqtt.isConnected, onConnectionReady, mqtt]);

    const handleConnect = async () => {
        await mqtt.connect();
    };

    const handleDisconnect = async () => {
        await mqtt.disconnect();
    };

    return (
        <Card className="p-6 bg-slate-950 border-slate-800">
            <div className="space-y-4">
                <div className="flex items-center justify-between">
                    <div>
                        <h2 className="text-xl font-bold text-slate-200 uppercase tracking-wide">
                            MQTT Connection
                        </h2>
                        <p className="text-sm text-slate-500 mt-1">
                            Drone Communication Protocol
                        </p>
                    </div>
                    <Badge
                        variant={mqtt.isConnected ? "default" : "secondary"}
                        className="uppercase"
                    >
                        {mqtt.isConnected ? (
                            <>
                                <Activity className="mr-2 h-3 w-3" />
                                Connected
                            </>
                        ) : (
                            <>
                                <WifiOff className="mr-2 h-3 w-3" />
                                Disconnected
                            </>
                        )}
                    </Badge>
                </div>

                <div className="flex gap-3">
                    {!mqtt.isConnected ? (
                        <Button
                            onClick={handleConnect}
                            disabled={mqtt.isConnecting}
                            className="flex-1 bg-emerald-600 hover:bg-emerald-700"
                        >
                            <Radio className="mr-2 h-4 w-4" />
                            {mqtt.isConnecting
                                ? "Connecting..."
                                : "Connect MQTT"}
                        </Button>
                    ) : (
                        <Button
                            onClick={handleDisconnect}
                            variant="destructive"
                            className="flex-1"
                        >
                            <Wifi className="mr-2 h-4 w-4" />
                            Disconnect
                        </Button>
                    )}
                </div>

                {mqtt.error && (
                    <div className="p-3 bg-red-950 border border-red-800 rounded-lg">
                        <p className="text-sm text-red-200">⚠️ {mqtt.error}</p>
                    </div>
                )}

                {mqtt.isConnected && (
                    <div className="space-y-3">
                        <div className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                            <div className="flex items-center justify-between">
                                <span className="text-xs text-slate-400 uppercase tracking-wide">
                                    Messages Received
                                </span>
                                <span className="text-lg font-bold text-emerald-400 font-mono">
                                    {mqtt.messageCount}
                                </span>
                            </div>
                        </div>


                        {mqtt.videoFrame && (
                            <div className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                                <div className="flex items-center justify-between">
                                    <span className="text-xs text-slate-400 uppercase tracking-wide">
                                        Video Stream
                                    </span>
                                    <span className="text-xs text-emerald-400 font-mono">
                                        Active
                                    </span>
                                </div>
                            </div>
                        )}
                    </div>
                )}
            </div>
        </Card>
    );
}
