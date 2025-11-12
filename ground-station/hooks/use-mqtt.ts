"use client";

import { useState, useCallback, useRef, useEffect } from "react";
import mqtt, { MqttClient } from "mqtt";
import { MQTT_CONFIG, MQTT_TOPICS } from "@/lib/config";

export interface DroneCommand {
    action:
        | "takeoff"
        | "land"
        | "move"
        | "auto_land"
        | "enable_centering"
        | "disable_centering";
    value?: string;
    altitude?: number;
    destination?: string;
}

export interface ArUcoDetection {
    detected: boolean;
    marker_id?: number;
    distance?: number;
    angle?: number;
    [key: string]: unknown;
}

export interface VideoFrame {
    data: string;
    timestamp: number;
}

export interface MQTTState {
    isConnected: boolean;
    isConnecting: boolean;
    error: string | null;
    client: MqttClient | null;
    messageCount: number;
    videoFrame: VideoFrame | null;
    arUcoDetection: ArUcoDetection | null;
}

export interface UseMQTTReturn extends MQTTState {
    connect: () => Promise<void>;
    disconnect: () => Promise<void>;
    sendCommand: (command: DroneCommand) => Promise<boolean>;
    sendTakeoff: (altitude?: number) => Promise<boolean>;
    sendLand: () => Promise<boolean>;
    sendMove: (destination: string) => Promise<boolean>;
    sendAutoLand: () => Promise<boolean>;
    sendEnableCentering: () => Promise<boolean>;
    sendDisableCentering: () => Promise<boolean>;
}

export function useMQTT(): UseMQTTReturn {
    const [state, setState] = useState<MQTTState>({
        isConnected: false,
        isConnecting: false,
        error: null,
        client: null,
        messageCount: 0,
        videoFrame: null,
        arUcoDetection: null,
    });

    const clientRef = useRef<MqttClient | null>(null);
    const frameCountRef = useRef<number>(0);

    const handleMessage = useCallback((topic: string, payload: Buffer) => {
        try {
            if (topic === MQTT_TOPICS.STREAM) {
                const base64Image = payload.toString("utf-8");
                setState((prev) => ({
                    ...prev,
                    videoFrame: {
                        data: base64Image,
                        timestamp: Date.now(),
                    },
                }));
                frameCountRef.current += 1;
            } else if (topic === MQTT_TOPICS.ARUCO) {
                const detectionData: ArUcoDetection = JSON.parse(
                    payload.toString("utf-8")
                );
                setState((prev) => ({
                    ...prev,
                    arUcoDetection: detectionData,
                }));
            }

            setState((prev) => ({
                ...prev,
                messageCount: prev.messageCount + 1,
            }));
        } catch (error) {
            console.error(`Error processing message from ${topic}:`, error);
        }
    }, []);

    const connect = useCallback(async () => {
        setState((prev) => ({ ...prev, isConnecting: true, error: null }));

        try {
            const brokerUrl = `ws://${MQTT_CONFIG.BROKER_ADDRESS}:${MQTT_CONFIG.WS_PORT}`;

            const client = mqtt.connect(brokerUrl, {
                clientId: `ground-station-${Math.random()
                    .toString(16)
                    .slice(2, 8)}`,
                clean: true,
                reconnectPeriod: 5000,
                connectTimeout: 30000,
            });

            // Handle connection success
            client.on("connect", () => {
                console.log(
                    `✓ Connected to MQTT broker at ${MQTT_CONFIG.BROKER_ADDRESS}:${MQTT_CONFIG.WS_PORT}`
                );

                // Subscribe to topics
                client.subscribe(MQTT_TOPICS.STREAM, { qos: 0 }, (error) => {
                    if (error) {
                        console.error(
                            "Failed to subscribe to video stream:",
                            error
                        );
                    } else {
                        console.log(
                            `✓ Subscribed to video stream: ${MQTT_TOPICS.STREAM}`
                        );
                    }
                });

                client.subscribe(MQTT_TOPICS.ARUCO, { qos: 0 }, (error) => {
                    if (error) {
                        console.error(
                            "Failed to subscribe to ArUco detection:",
                            error
                        );
                    } else {
                        console.log(
                            `✓ Subscribed to ArUco detection: ${MQTT_TOPICS.ARUCO}`
                        );
                    }
                });

                setState((prev) => ({
                    ...prev,
                    isConnected: true,
                    isConnecting: false,
                    error: null,
                    client,
                }));
            });

            // Handle incoming messages
            client.on("message", handleMessage);

            // Handle errors
            client.on("error", (error) => {
                console.error("MQTT connection error:", error);
                setState((prev) => ({
                    ...prev,
                    error: error.message,
                    isConnected: false,
                    isConnecting: false,
                }));
            });

            // Handle disconnect
            client.on("close", () => {
                console.log("✓ Disconnected from MQTT broker");
                setState((prev) => ({
                    ...prev,
                    isConnected: false,
                }));
            });

            // Handle reconnect
            client.on("reconnect", () => {
                console.log("Attempting to reconnect to MQTT broker...");
            });

            clientRef.current = client;
        } catch (error) {
            setState((prev) => ({
                ...prev,
                isConnected: false,
                isConnecting: false,
                error:
                    error instanceof Error
                        ? error.message
                        : "Failed to connect",
            }));
        }
    }, [handleMessage]);

    /**
     * Disconnect from MQTT broker
     */
    const disconnect = useCallback(async () => {
        if (clientRef.current) {
            await clientRef.current.endAsync();
            clientRef.current = null;
        }

        setState({
            isConnected: false,
            isConnecting: false,
            error: null,
            client: null,
            messageCount: 0,
            videoFrame: null,
            arUcoDetection: null,
        });
    }, []);

    /**
     * Send a command to the drone via MQTT
     */
    const sendCommand = useCallback(
        async (command: DroneCommand): Promise<boolean> => {
            if (!clientRef.current || !state.isConnected) {
                console.error(
                    "Cannot send command: not connected to MQTT broker"
                );
                return false;
            }

            try {
                const payload = JSON.stringify(command);
                console.log(`→ Sending command: ${payload}`);

                await clientRef.current.publishAsync(
                    MQTT_TOPICS.COMMAND,
                    payload,
                    { qos: 1 }
                );
                console.log("✓ Command sent successfully");

                return true;
            } catch (error) {
                console.error("Error sending command:", error);
                return false;
            }
        },
        [state.isConnected]
    );

    /**
     * Send takeoff command
     */
    const sendTakeoff = useCallback(
        async (altitude: number = 5.0): Promise<boolean> => {
            return sendCommand({ action: "takeoff", altitude });
        },
        [sendCommand]
    );

    /**
     * Send land command
     */
    const sendLand = useCallback(async (): Promise<boolean> => {
        return sendCommand({ action: "land" });
    }, [sendCommand]);

    /**
     * Send move command
     */
    const sendMove = useCallback(
        async (destination: string): Promise<boolean> => {
            return sendCommand({ action: "move", value: destination });
        },
        [sendCommand]
    );

    /**
     * Send auto-land command (lands on detected ArUco marker)
     */
    const sendAutoLand = useCallback(async (): Promise<boolean> => {
        return sendCommand({ action: "auto_land" });
    }, [sendCommand]);

    /**
     * Send enable centering command
     */
    const sendEnableCentering = useCallback(async (): Promise<boolean> => {
        return sendCommand({ action: "enable_centering" });
    }, [sendCommand]);

    /**
     * Send disable centering command
     */
    const sendDisableCentering = useCallback(async (): Promise<boolean> => {
        return sendCommand({ action: "disable_centering" });
    }, [sendCommand]);

    /**
     * Cleanup on unmount
     */
    useEffect(() => {
        return () => {
            if (clientRef.current) {
                clientRef.current.end();
            }
        };
    }, []);

    return {
        ...state,
        connect,
        disconnect,
        sendCommand,
        sendTakeoff,
        sendLand,
        sendMove,
        sendAutoLand,
        sendEnableCentering,
        sendDisableCentering,
    };
}
