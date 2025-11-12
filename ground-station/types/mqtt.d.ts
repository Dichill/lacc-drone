export interface MQTTConnectionOptions {
    clientId?: string;
    clean?: boolean;
    reconnectPeriod?: number;
    connectTimeout?: number;
    username?: string;
    password?: string;
}

export interface MQTTMessage {
    topic: string;
    payload: Buffer | string;
    qos: 0 | 1 | 2;
    retain: boolean;
}

export type DroneCommandAction =
    | "takeoff"
    | "land"
    | "move"
    | "auto_land"
    | "enable_centering"
    | "disable_centering"
    | "arm"
    | "disarm"
    | "emergency_stop";

export interface BaseDroneCommand {
    action: DroneCommandAction;
    timestamp?: number;
}

export interface TakeoffCommand extends BaseDroneCommand {
    action: "takeoff";
    altitude: number;
}

export interface MoveCommand extends BaseDroneCommand {
    action: "move";
    value: string;
    destination?: string;
}

export interface SimpleCommand extends BaseDroneCommand {
    action:
        | "land"
        | "auto_land"
        | "enable_centering"
        | "disable_centering"
        | "arm"
        | "disarm"
        | "emergency_stop";
}

export type DroneCommand = TakeoffCommand | MoveCommand | SimpleCommand;

export interface ArUcoDetection {
    detected: boolean;
    marker_id?: number;
    distance?: number;
    angle?: number;
    x?: number;
    y?: number;
    corners?: Array<{ x: number; y: number }>;
    [key: string]: unknown;
}

export interface VideoFrame {
    data: string;
    timestamp: number;
    width?: number;
    height?: number;
    format?: string;
}

export interface TelemetryData {
    altitude: number;
    groundSpeed: number;
    distanceToWaypoint: number;
    yaw: number;
    verticalSpeed: number;
    latitude?: number;
    longitude?: number;
    battery?: number;
    mode?: string;
    armed?: boolean;
}

export type MQTTTopic =
    | "drone/commands"
    | "camera/stream"
    | "drone/aruco_detection";

export type MQTTQoS = 0 | 1 | 2;
