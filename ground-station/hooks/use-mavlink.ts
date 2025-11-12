"use client";

import { useState, useCallback, useRef } from "react";

export interface MAVLinkMessage {
    header: {
        magic: number;
        len: number;
        seq: number;
        sysid: number;
        compid: number;
        msgid: number;
    };
    payload: Record<string, unknown>;
    timestamp: number;
}

export interface UseMAVLinkReturn {
    lastMessage: MAVLinkMessage | null;
    messageCount: number;
    processData: (data: Uint8Array) => void;
    reset: () => void;
}

export function useMAVLink(): UseMAVLinkReturn {
    const [lastMessage, setLastMessage] = useState<MAVLinkMessage | null>(null);
    const [messageCount, setMessageCount] = useState(0);
    const bufferRef = useRef<number[]>([]);

    const processData = useCallback((data: Uint8Array) => {
        try {
            for (let i = 0; i < data.length; i++) {
                const byte = data[i];
                bufferRef.current.push(byte);

                if (bufferRef.current.length < 8) continue;

                const magic = bufferRef.current[0];
                if (magic !== 0xfd && magic !== 0xfe) {
                    bufferRef.current.shift();
                    continue;
                }

                const isV2 = magic === 0xfd;
                const headerSize = isV2 ? 10 : 6;

                if (bufferRef.current.length < headerSize) continue;

                const payloadLen = bufferRef.current[1];
                const totalLen = headerSize + payloadLen + (isV2 ? 2 : 2);

                if (bufferRef.current.length < totalLen) continue;

                const buffer = new Uint8Array(
                    bufferRef.current.slice(0, totalLen)
                );
                bufferRef.current = bufferRef.current.slice(totalLen);

                const seq = buffer[isV2 ? 2 : 2];
                const sysid = buffer[isV2 ? 3 : 3];
                const compid = buffer[isV2 ? 4 : 4];
                const msgid = isV2
                    ? buffer[5] | (buffer[6] << 8) | (buffer[7] << 16)
                    : buffer[5];

                const payloadStart = headerSize;
                const payload: Record<string, unknown> = {};

                if (msgid === 33) {
                    const view = new DataView(
                        buffer.buffer,
                        buffer.byteOffset + payloadStart
                    );
                    payload.time_boot_ms = view.getUint32(0, true);
                    payload.lat = view.getInt32(4, true);
                    payload.lon = view.getInt32(8, true);
                    payload.alt = view.getInt32(12, true);
                    payload.relative_alt = view.getInt32(16, true);
                    payload.vx = view.getInt16(20, true);
                    payload.vy = view.getInt16(22, true);
                    payload.vz = view.getInt16(24, true);
                    payload.hdg = view.getUint16(26, true);
                } else if (msgid === 30) {
                    const view = new DataView(
                        buffer.buffer,
                        buffer.byteOffset + payloadStart
                    );
                    payload.time_boot_ms = view.getUint32(0, true);
                    payload.roll = view.getFloat32(4, true);
                    payload.pitch = view.getFloat32(8, true);
                    payload.yaw = view.getFloat32(12, true);
                    payload.rollspeed = view.getFloat32(16, true);
                    payload.pitchspeed = view.getFloat32(20, true);
                    payload.yawspeed = view.getFloat32(24, true);
                } else if (msgid === 62) {
                    const view = new DataView(
                        buffer.buffer,
                        buffer.byteOffset + payloadStart
                    );
                    payload.wp_dist = view.getUint16(0, true);
                    payload.nav_roll = view.getFloat32(2, true);
                    payload.nav_pitch = view.getFloat32(6, true);
                    payload.nav_bearing = view.getInt16(10, true);
                    payload.target_bearing = view.getInt16(12, true);
                    payload.wp_num = view.getUint16(14, true);
                    payload.alt_error = view.getFloat32(16, true);
                    payload.aspd_error = view.getFloat32(20, true);
                    payload.xtrack_error = view.getFloat32(24, true);
                }

                setLastMessage({
                    header: {
                        magic,
                        len: payloadLen,
                        seq,
                        sysid,
                        compid,
                        msgid,
                    },
                    payload,
                    timestamp: Date.now(),
                });
                setMessageCount((prev) => prev + 1);
            }
        } catch (error) {
            console.error("MAVLink parse error:", error);
        }
    }, []);

    const reset = useCallback(() => {
        setLastMessage(null);
        setMessageCount(0);
    }, []);

    return {
        lastMessage,
        messageCount,
        processData,
        reset,
    };
}
