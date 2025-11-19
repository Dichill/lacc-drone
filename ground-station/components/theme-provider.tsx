"use client";

import React, { createContext, useContext, useEffect, useState } from "react";

/**
 * Type definition for theme mode
 */
type Theme = "light" | "dark";

/**
 * Theme context value interface
 */
interface ThemeContextValue {
    theme: Theme;
    setTheme: (theme: Theme) => void;
    toggleTheme: () => void;
}

/**
 * Theme context for managing application theme state
 */
const ThemeContext = createContext<ThemeContextValue | undefined>(undefined);

/**
 * Theme provider component that manages theme state and persistence
 * @param props - Component props
 * @param props.children - Child components to wrap
 * @param props.defaultTheme - Default theme to use (defaults to "dark")
 */
export function ThemeProvider({
    children,
    defaultTheme = "dark",
}: {
    children: React.ReactNode;
    defaultTheme?: Theme;
}) {
    const [theme, setThemeState] = useState<Theme>(defaultTheme);
    const [mounted, setMounted] = useState(false);

    // Load theme from localStorage on mount
    useEffect(() => {
        const savedTheme = localStorage.getItem("theme") as Theme | null;
        if (savedTheme && (savedTheme === "light" || savedTheme === "dark")) {
            setThemeState(savedTheme);
        }
        setMounted(true);
    }, []);

    // Update document class and localStorage when theme changes
    useEffect(() => {
        if (!mounted) return;

        const root = document.documentElement;
        if (theme === "dark") {
            root.classList.add("dark");
        } else {
            root.classList.remove("dark");
        }
        localStorage.setItem("theme", theme);
    }, [theme, mounted]);

    /**
     * Set the theme and persist to localStorage
     * @param newTheme - The theme to set
     */
    const setTheme = (newTheme: Theme) => {
        setThemeState(newTheme);
    };

    /**
     * Toggle between light and dark themes
     */
    const toggleTheme = () => {
        setThemeState((prev) => (prev === "light" ? "dark" : "light"));
    };

    const value: ThemeContextValue = {
        theme,
        setTheme,
        toggleTheme,
    };

    return (
        <ThemeContext.Provider value={value}>{children}</ThemeContext.Provider>
    );
}

/**
 * Hook to access theme context
 * @returns Theme context value
 * @throws Error if used outside ThemeProvider
 */
export function useTheme() {
    const context = useContext(ThemeContext);
    if (context === undefined) {
        throw new Error("useTheme must be used within a ThemeProvider");
    }
    return context;
}
