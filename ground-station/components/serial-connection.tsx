"use client";

import { useEffect } from "react";
import { Card } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Badge } from "@/components/ui/badge";
import { Usb, WifiOff, Activity } from "lucide-react";
import { useSerialPort } from "@/hooks/use-serial-port";
import { useMAVLink } from "@/hooks/use-mavlink";

interface SerialConnectionProps {
    onTelemetryUpdate?: (message: Record<string, unknown>) => void;
}

export function SerialConnection({ onTelemetryUpdate }: SerialConnectionProps) {
    const { isConnected, isConnecting, error, connect, disconnect, onData } =
        useSerialPort(57600);
    const { lastMessage, messageCount, processData, reset } = useMAVLink();

    useEffect(() => {
        onData(processData);
    }, [onData, processData]);

    useEffect(() => {
        if (lastMessage && onTelemetryUpdate) {
            onTelemetryUpdate(lastMessage.payload);
        }
    }, [lastMessage, onTelemetryUpdate]);

    const handleConnect = async () => {
        reset();
        await connect();
    };

    const handleDisconnect = async () => {
        await disconnect();
        reset();
    };

    return (
        <Card className="p-6 bg-slate-950 border-slate-800">
            <div className="space-y-4">
                <div className="flex items-center justify-between">
                    <div>
                        <h2 className="text-xl font-bold text-slate-200 uppercase tracking-wide">
                            Serial Connection
                        </h2>
                        <p className="text-sm text-slate-500 mt-1">
                            MAVLink Telemetry
                        </p>
                    </div>
                    <Badge
                        variant={isConnected ? "default" : "secondary"}
                        className="uppercase"
                    >
                        {isConnected ? (
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
                    {!isConnected ? (
                        <Button
                            onClick={handleConnect}
                            disabled={isConnecting}
                            className="flex-1 bg-emerald-600 hover:bg-emerald-700"
                        >
                            <Usb className="mr-2 h-4 w-4" />
                            {isConnecting ? "Connecting..." : "Connect Serial"}
                        </Button>
                    ) : (
                        <Button
                            onClick={handleDisconnect}
                            variant="destructive"
                            className="flex-1"
                        >
                            Disconnect
                        </Button>
                    )}
                </div>

                {error && (
                    <div className="p-3 bg-red-950 border border-red-800 rounded-lg">
                        <p className="text-sm text-red-200">⚠️ {error}</p>
                    </div>
                )}

                {isConnected && (
                    <div className="space-y-3">
                        <div className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                            <div className="flex items-center justify-between">
                                <span className="text-xs text-slate-400 uppercase tracking-wide">
                                    Messages Received
                                </span>
                                <span className="text-lg font-bold text-emerald-400 font-mono">
                                    {messageCount}
                                </span>
                            </div>
                        </div>

                        {lastMessage && (
                            <div className="p-3 bg-slate-900 rounded-lg border border-slate-800">
                                <div className="space-y-2">
                                    <div className="flex items-center justify-between">
                                        <span className="text-xs text-slate-400 uppercase tracking-wide">
                                            Last Message
                                        </span>
                                        <span className="text-xs text-slate-500 font-mono">
                                            ID: {lastMessage.header.msgid}
                                        </span>
                                    </div>
                                    <div className="max-h-40 overflow-y-auto">
                                        <pre className="text-xs text-slate-300 font-mono whitespace-pre-wrap break-words">
                                            {JSON.stringify(
                                                lastMessage.payload,
                                                null,
                                                2
                                            )}
                                        </pre>
                                    </div>
                                </div>
                            </div>
                        )}
                    </div>
                )}
            </div>
        </Card>
    );
}
