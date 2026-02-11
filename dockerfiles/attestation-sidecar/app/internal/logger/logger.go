package logger

import (
	"fmt"
	"os"
	"strings"
	"time"
)

// ANSI color codes matching your Python implementation
const (
	Green   = "\033[92m"
	Yellow  = "\033[93m"
	Blue    = "\033[94m"
	Magenta = "\033[95m"
	Cyan    = "\033[96m"
	Red     = "\033[91m"
	Reset   = "\033[0m"
)

// LogLevel type equivalent to Python IntEnum
type LogLevel int

const (
	DebugLevel LogLevel = 10
	InfoLevel  LogLevel = 20
	WarnLevel  LogLevel = 30
	ErrorLevel LogLevel = 40
	NoneLevel  LogLevel = 100
)

var currentLevel LogLevel

// init runs automatically on package import, matching your global _CURRENT_LEVEL logic
func init() {
	currentLevel = parseLevel(os.Getenv("LOG_LEVEL"))
}

func parseLevel(envValue string) LogLevel {
	v := strings.ToUpper(strings.TrimSpace(envValue))
	switch v {
	case "DEBUG":
		return DebugLevel
	case "INFO":
		return InfoLevel
	case "WARN", "WARNING":
		return WarnLevel
	case "ERROR":
		return ErrorLevel
	case "NONE":
		return NoneLevel
	default:
		return InfoLevel
	}
}

// Format logic matching YYYY-MM-DD HH:MM:SS
func format(levelStr, color, msg string) string {
	timestamp := time.Now().Format("2006-01-02 15:04:05")
	return fmt.Sprintf("%s - %s%s%s - %s", timestamp, color, levelStr, Reset, msg)
}

func _print(line string) {
	// Go's Fprintln to Stdout is synchronous and effective for "flush immediately" logic
	fmt.Fprintln(os.Stdout, line)
}

// Public Logging Functions

func Debug(formatStr string, args ...interface{}) {
	if currentLevel <= DebugLevel {
		msg := fmt.Sprintf(formatStr, args...)
		_print(format("DEBUG", Cyan, msg))
	}
}

func Info(formatStr string, args ...interface{}) {
	if currentLevel <= InfoLevel {
		msg := fmt.Sprintf(formatStr, args...)
		_print(format("INFO", Green, msg))
	}
}

func Warn(formatStr string, args ...interface{}) {
	if currentLevel <= WarnLevel {
		msg := fmt.Sprintf(formatStr, args...)
		_print(format("WARN", Yellow, msg))
	}
}

func Error(formatStr string, args ...interface{}) {
	if currentLevel <= ErrorLevel {
		msg := fmt.Sprintf(formatStr, args...)
		_print(format("ERROR", Red, msg))
	}
}

// Colored output helpers
func GreenText(s string) string {
	return fmt.Sprintf("%s%s%s", Green, s, Reset)
}

func YellowText(s string) string {
	return fmt.Sprintf("%s%s%s", Yellow, s, Reset)
}