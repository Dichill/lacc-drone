"use client";

import { useState, useCallback, useRef, useEffect } from "react";

export interface SerialPortState {
    isConnected: boolean;
    isConnecting: boolean;
    error: string | null;
    port: SerialPort | null;
}

export interface UseSerialPortReturn extends SerialPortState {
    connect: () => Promise<void>;
    disconnect: () => Promise<void>;
    onData: (callback: (data: Uint8Array) => void) => void;
}

export function useSerialPort(baudRate: number = 57600): UseSerialPortReturn {
    const [state, setState] = useState<SerialPortState>({
        isConnected: false,
        isConnecting: false,
        error: null,
        port: null,
    });

    const readerRef = useRef<ReadableStreamDefaultReader<Uint8Array> | null>(
        null
    );
    const dataCallbackRef = useRef<((data: Uint8Array) => void) | null>(null);
    const isReadingRef = useRef(false);

    const readLoop = useCallback(
        async (reader: ReadableStreamDefaultReader<Uint8Array>) => {
            isReadingRef.current = true;
            try {
                while (isReadingRef.current) {
                    const { value, done } = await reader.read();
                    if (done) {
                        break;
                    }
                    if (value && dataCallbackRef.current) {
                        dataCallbackRef.current(value);
                    }
                }
            } catch (error) {
                if (isReadingRef.current) {
                    setState((prev) => ({
                        ...prev,
                        error:
                            error instanceof Error
                                ? error.message
                                : "Read error",
                        isConnected: false,
                    }));
                }
            }
        },
        []
    );

    const connect = useCallback(async () => {
        if (typeof navigator === "undefined" || !navigator.serial) {
            setState((prev) => ({
                ...prev,
                error: "Web Serial API not supported in this browser",
            }));
            return;
        }

        setState((prev) => ({ ...prev, isConnecting: true, error: null }));

        try {
            const port = await navigator.serial.requestPort();
            await port.open({
                baudRate,
                dataBits: 8,
                stopBits: 1,
                parity: "none",
            });

            if (port.readable) {
                const reader = port.readable.getReader();
                readerRef.current = reader;
                readLoop(reader);
            }

            setState({
                isConnected: true,
                isConnecting: false,
                error: null,
                port,
            });
        } catch (error) {
            setState({
                isConnected: false,
                isConnecting: false,
                error:
                    error instanceof Error
                        ? error.message
                        : "Failed to connect",
                port: null,
            });
        }
    }, [baudRate, readLoop]);

    const disconnect = useCallback(async () => {
        isReadingRef.current = false;

        if (readerRef.current) {
            try {
                await readerRef.current.cancel();
                readerRef.current.releaseLock();
            } catch (error) {
                console.error("Error releasing reader:", error);
            }
            readerRef.current = null;
        }

        if (state.port) {
            try {
                await state.port.close();
            } catch (error) {
                console.error("Error closing port:", error);
            }
        }

        setState({
            isConnected: false,
            isConnecting: false,
            error: null,
            port: null,
        });
    }, [state.port]);

    const onData = useCallback((callback: (data: Uint8Array) => void) => {
        dataCallbackRef.current = callback;
    }, []);

    useEffect(() => {
        return () => {
            isReadingRef.current = false;
            if (readerRef.current) {
                readerRef.current.cancel().catch(() => {});
            }
        };
    }, []);

    return {
        ...state,
        connect,
        disconnect,
        onData,
    };
}
