import threading
import subprocess
import time


class ShellCommandHotkey:
    def __init__(self, trigger_key, command, cooldown_sec=0.0, process_factory=None):
        self._lock = threading.Lock()
        self.trigger_key = self._normalize_key(trigger_key)
        self.command = '' if command is None else str(command).strip()
        self.cooldown_sec = max(0.0, float(cooldown_sec))
        self.process_factory = process_factory or self._default_process_factory
        self._process = None
        self._last_trigger_time = None

    def is_enabled(self):
        return self.trigger_key is not None and bool(self.command)

    def matches(self, key):
        return self.trigger_key is not None and self._normalize_key(key) == self.trigger_key

    def is_running(self):
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def trigger(self, now=None):
        timestamp = time.monotonic() if now is None else float(now)

        with self._lock:
            if not self.is_enabled():
                return 'disabled'

            if self._process is not None and self._process.poll() is None:
                return 'running'

            if self._last_trigger_time is not None:
                if timestamp - self._last_trigger_time < self.cooldown_sec:
                    return 'cooldown'

            self._process = self.process_factory(self.command)
            self._last_trigger_time = timestamp
            return 'started'

    def consume_exit_code(self):
        with self._lock:
            if self._process is None:
                return None

            return_code = self._process.poll()
            if return_code is None:
                return None

            self._process = None
            return return_code

    @staticmethod
    def _normalize_key(key):
        if key is None:
            return None

        normalized_key = str(key).strip().lower()
        if len(normalized_key) != 1:
            return None

        return normalized_key

    @staticmethod
    def _default_process_factory(command):
        return subprocess.Popen(
            ['bash', '-lc', command],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
