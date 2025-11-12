export const SIMULATION_MODE: boolean = true;

/**
 * MQTT Broker Configuration
 */
export const MQTT_CONFIG = {
    /**
     * Broker address
     * - localhost for simulation
     * - 192.168.0.163 for real drone
     */
    BROKER_ADDRESS: SIMULATION_MODE ? "localhost" : "192.168.0.163",

    /**
     * Broker port
     * Default: 1883 for MQTT, 9001 for WebSocket
     */
    BROKER_PORT: 1883,

    /**
     * WebSocket port (for browser connections)
     */
    WS_PORT: 9001,
} as const;

/**
 * MQTT Topic Configuration
 */
export const MQTT_TOPICS = {
    COMMAND: "drone/commands",
    STREAM: "camera/stream",
    ARUCO: "drone/aruco_detection",
} as const;

/**
 * Default flight parameters
 */
export const FLIGHT_DEFAULTS = {
    TAKEOFF_ALTITUDE: 5.0,
    MAX_ALTITUDE: 50.0,
    MIN_ALTITUDE: 1.0,
} as const;

export function getMQTTWebSocketUrl(): string {
    return `ws://${MQTT_CONFIG.BROKER_ADDRESS}:${MQTT_CONFIG.WS_PORT}`;
}

export function getMQTTUrl(): string {
    return `mqtt://${MQTT_CONFIG.BROKER_ADDRESS}:${MQTT_CONFIG.BROKER_PORT}`;
}
