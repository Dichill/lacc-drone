import json
import datetime
import os
from typing import Any, Dict, Optional, List
from enum import Enum
import sys


class LogLevel(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class DroneLogger:
    def __init__(self, log_dir: str = "./logs", console_output: bool = True):
        self.log_dir: str = log_dir
        self.console_output: bool = console_output
        self.session_start: datetime.datetime = datetime.datetime.now()
        self.session_id: str = self.session_start.strftime("%Y-%m-%d_%H-%M-%S")
        self.log_file: str = os.path.join(self.log_dir, f"log_{self.session_id}.jsonl")
        self.summary_file: str = os.path.join(self.log_dir, f"summary_{self.session_id}.json")
        self.log_buffer: List[Dict[str, Any]] = []
        
        os.makedirs(self.log_dir, exist_ok=True)
        
        self._write_session_start()
    
    def _write_session_start(self) -> None:
        start_entry: Dict[str, Any] = {
            "timestamp": self.session_start.isoformat(),
            "event": "SESSION_START",
            "session_id": self.session_id
        }
        self._write_to_file(start_entry)
    
    def _write_to_file(self, entry: Dict[str, Any]) -> None:
        try:
            with open(self.log_file, "a") as f:
                f.write(json.dumps(entry) + "\n")
        except Exception as e:
            print(f"Failed to write log: {e}", file=sys.stderr)
    
    def _format_console_message(self, level: LogLevel, message: str, metadata: Optional[Dict[str, Any]] = None) -> str:
        timestamp: str = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        base_msg: str = f"[{timestamp}] [{level.value}] {message}"
        
        if metadata:
            meta_str: str = " | ".join([f"{k}={v}" for k, v in metadata.items()])
            return f"{base_msg} | {meta_str}"
        
        return base_msg
    
    def _log(self, level: LogLevel, message: str, metadata: Optional[Dict[str, Any]] = None) -> None:
        timestamp: datetime.datetime = datetime.datetime.now()
        
        log_entry: Dict[str, Any] = {
            "timestamp": timestamp.isoformat(),
            "level": level.value,
            "message": message
        }
        
        if metadata:
            log_entry["metadata"] = metadata
        
        self.log_buffer.append(log_entry)
        self._write_to_file(log_entry)
        
        if self.console_output:
            console_msg: str = self._format_console_message(level, message, metadata)
            
            if level in [LogLevel.ERROR, LogLevel.CRITICAL]:
                print(console_msg, file=sys.stderr)
            else:
                print(console_msg)
    
    def debug(self, message: str, **kwargs: Any) -> None:
        metadata: Optional[Dict[str, Any]] = kwargs if kwargs else None
        self._log(LogLevel.DEBUG, message, metadata)
    
    def info(self, message: str, **kwargs: Any) -> None:
        metadata: Optional[Dict[str, Any]] = kwargs if kwargs else None
        self._log(LogLevel.INFO, message, metadata)
    
    def warning(self, message: str, **kwargs: Any) -> None:
        metadata: Optional[Dict[str, Any]] = kwargs if kwargs else None
        self._log(LogLevel.WARNING, message, metadata)
    
    def error(self, message: str, **kwargs: Any) -> None:
        metadata: Optional[Dict[str, Any]] = kwargs if kwargs else None
        self._log(LogLevel.ERROR, message, metadata)
    
    def critical(self, message: str, **kwargs: Any) -> None:
        metadata: Optional[Dict[str, Any]] = kwargs if kwargs else None
        self._log(LogLevel.CRITICAL, message, metadata)
    
    def metric(self, name: str, value: Any, unit: Optional[str] = None, **kwargs: Any) -> None:
        metadata: Dict[str, Any] = {
            "metric_name": name,
            "metric_value": value,
            **kwargs
        }
        
        if unit:
            metadata["unit"] = unit
        
        message: str = f"Metric: {name} = {value}"
        if unit:
            message = f"{message} {unit}"
        
        self._log(LogLevel.INFO, message, metadata)
    
    def event(self, event_name: str, **kwargs: Any) -> None:
        metadata: Dict[str, Any] = {
            "event_name": event_name,
            **kwargs
        }
        
        self._log(LogLevel.INFO, f"Event: {event_name}", metadata)
    
    def generate_summary(self) -> Dict[str, Any]:
        level_counts: Dict[str, int] = {level.value: 0 for level in LogLevel}
        events: List[str] = []
        metrics: List[Dict[str, Any]] = []
        errors: List[Dict[str, Any]] = []
        
        for entry in self.log_buffer:
            level: str = entry.get("level", "")
            if level in level_counts:
                level_counts[level] += 1
            
            if level in ["ERROR", "CRITICAL"]:
                errors.append({
                    "timestamp": entry["timestamp"],
                    "message": entry["message"],
                    "metadata": entry.get("metadata")
                })
            
            metadata: Optional[Dict[str, Any]] = entry.get("metadata")
            if metadata:
                if "event_name" in metadata:
                    events.append(metadata["event_name"])
                
                if "metric_name" in metadata:
                    metrics.append({
                        "timestamp": entry["timestamp"],
                        "name": metadata["metric_name"],
                        "value": metadata["metric_value"],
                        "unit": metadata.get("unit")
                    })
        
        session_end: datetime.datetime = datetime.datetime.now()
        duration: float = (session_end - self.session_start).total_seconds()
        
        summary: Dict[str, Any] = {
            "session_id": self.session_id,
            "session_start": self.session_start.isoformat(),
            "session_end": session_end.isoformat(),
            "duration_seconds": duration,
            "total_logs": len(self.log_buffer),
            "log_levels": level_counts,
            "events": events,
            "metrics": metrics,
            "errors": errors
        }
        
        try:
            with open(self.summary_file, "w") as f:
                json.dump(summary, f, indent=2)
        except Exception as e:
            print(f"Failed to write summary: {e}", file=sys.stderr)
        
        return summary
    
    def close(self) -> None:
        end_entry: Dict[str, Any] = {
            "timestamp": datetime.datetime.now().isoformat(),
            "event": "SESSION_END",
            "session_id": self.session_id
        }
        self._write_to_file(end_entry)
        
        summary: Dict[str, Any] = self.generate_summary()
        
        if self.console_output:
            print(f"\n{'='*60}")
            print(f"Session Summary - {self.session_id}")
            print(f"{'='*60}")
            print(f"Duration: {summary['duration_seconds']:.2f}s")
            print(f"Total Logs: {summary['total_logs']}")
            print(f"Errors: {summary['log_levels']['ERROR'] + summary['log_levels']['CRITICAL']}")
            print(f"Log file: {self.log_file}")
            print(f"Summary file: {self.summary_file}")
            print(f"{'='*60}")


_global_logger: Optional[DroneLogger] = None


def get_logger() -> DroneLogger:
    global _global_logger
    if _global_logger is None:
        _global_logger = DroneLogger()
    return _global_logger


def init_logger(log_dir: str = "./logs", console_output: bool = True) -> DroneLogger:
    global _global_logger
    _global_logger = DroneLogger(log_dir=log_dir, console_output=console_output)
    return _global_logger


def close_logger() -> None:
    global _global_logger
    if _global_logger is not None:
        _global_logger.close()
        _global_logger = None

