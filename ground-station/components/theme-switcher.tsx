"use client";

import React, { useState, useEffect } from "react";
import { Moon, Sun } from "lucide-react";
import { useTheme } from "@/components/theme-provider";
import { Button } from "@/components/ui/button";

/**
 * Theme switcher component that toggles between light and dark modes
 */
export function ThemeSwitcher() {
    const { theme, toggleTheme } = useTheme();
    const [mounted, setMounted] = useState(false);

    // Only render the theme switcher after mounting to avoid hydration mismatch
    useEffect(() => {
        setMounted(true);
    }, []);

    if (!mounted) {
        return (
            <Button
                variant="outline"
                size="icon"
                className="relative overflow-hidden border-slate-300 dark:border-slate-700"
                disabled
                aria-label="Loading theme"
            >
                <Sun className="h-[1.2rem] w-[1.2rem]" />
                <span className="sr-only">Loading theme</span>
            </Button>
        );
    }

    return (
        <Button
            variant="outline"
            size="icon"
            onClick={toggleTheme}
            className="relative overflow-hidden border-slate-300 dark:border-slate-700"
            aria-label={`Switch to ${
                theme === "light" ? "dark" : "light"
            } mode`}
        >
            {/* Sun icon for light mode */}
            <Sun
                className={`h-[1.2rem] w-[1.2rem] transition-all ${
                    theme === "light"
                        ? "rotate-0 scale-100"
                        : "rotate-90 scale-0"
                } absolute`}
            />
            {/* Moon icon for dark mode */}
            <Moon
                className={`h-[1.2rem] w-[1.2rem] transition-all ${
                    theme === "dark"
                        ? "rotate-0 scale-100"
                        : "-rotate-90 scale-0"
                } absolute`}
            />
            <span className="sr-only">Toggle theme</span>
        </Button>
    );
}
