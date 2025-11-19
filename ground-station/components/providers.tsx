"use client";

import React from "react";
import { ThemeProvider } from "@/components/theme-provider";

/**
 * Providers component that wraps the app with necessary context providers
 * This is a client component that can be imported into the server-side layout
 * @param props - Component props
 * @param props.children - Child components to wrap
 */
export function Providers({ children }: { children: React.ReactNode }) {
    return <ThemeProvider defaultTheme="dark">{children}</ThemeProvider>;
}
