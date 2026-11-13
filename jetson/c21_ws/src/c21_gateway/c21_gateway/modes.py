"""Mode manager: starts/stops feature launch files as child process groups.

Modes in the same exclusive group replace each other (e.g. mapping vs navigation),
so the Jetson never runs two heavy pipelines or two motion sources at once.
"""
import os
import re
import signal
import subprocess
import threading
import time

import yaml


class ModeManager:
    def __init__(self, config_path, logger):
        self.log = logger
        self.modes = {}
        self.procs = {}          # name -> Popen
        self.errors = {}
        self.started_args = {}
        self.lock = threading.Lock()
        if config_path and os.path.exists(config_path):
            with open(config_path, encoding='utf-8') as f:
                self.modes = (yaml.safe_load(f) or {}).get('modes', {})
        self.log.info(f'modes: {sorted(self.modes)}')

    def catalogue(self):
        return [{'name': n, 'label': m.get('label', n), 'group': m.get('group'),
                 'motion': bool(m.get('motion', False))} for n, m in self.modes.items()]

    def args_of(self, name):
        with self.lock:
            return dict(self.started_args.get(name, {})) if name in self.procs else None

    def status(self):
        with self.lock:
            self._reap()
            return {'running': sorted(self.procs), 'errors': dict(self.errors)}

    def running_motion(self):
        with self.lock:
            return any(self.modes.get(n, {}).get('motion') for n in self.procs)

    def _reap(self):
        for n, p in list(self.procs.items()):
            rc = p.poll()
            if rc is not None:
                del self.procs[n]
                if rc != 0:
                    self.errors[n] = f'exited with code {rc}'
                    self.log.warn(f'mode {n} exited rc={rc}')

    def start(self, name, args=None):
        if name not in self.modes:
            return False, f'unknown mode {name}'
        m = self.modes[name]
        with self.lock:
            self._reap()
            if name in self.procs:
                return True, 'already running'
            group = m.get('group')
            to_stop = [n for n in self.procs
                       if n != name and group and self.modes.get(n, {}).get('group') == group]
        for n in to_stop:
            self.stop(n)
        cmd = list(m['cmd'])
        for k, v in (args or {}).items():
            if k in m.get('args', []):
                v = str(v)
                if not re.fullmatch(r'[\w\-.]{1,64}', v) or '..' in v:
                    return False, f'bad value for {k}'
                cmd.append(f'{k}:={v}')
        missing = [k for k in m.get('args', []) if k not in (args or {})]
        if missing:
            return False, f'missing {", ".join(missing)}'
        log_dir = os.path.expanduser('~/.ros/c21_modes')
        os.makedirs(log_dir, exist_ok=True)
        logf = open(os.path.join(log_dir, f'{name}.log'), 'ab')
        logf.write(f'\n==== {time.ctime()} {" ".join(cmd)}\n'.encode())
        p = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True)
        with self.lock:
            self.procs[name] = p
            self.started_args[name] = dict(args or {})
            self.errors.pop(name, None)
        self.log.info(f'mode {name} started pid={p.pid}')
        return True, 'started'

    def stop(self, name, timeout=8.0):
        with self.lock:
            p = self.procs.pop(name, None)
        if p is None:
            return True, 'not running'
        try:
            os.killpg(p.pid, signal.SIGINT)
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=2.0)
        except ProcessLookupError:
            pass
        self.log.info(f'mode {name} stopped')
        return True, 'stopped'

    def stop_motion(self):
        for n in list(self.procs):
            if self.modes.get(n, {}).get('motion'):
                self.stop(n)

    def stop_all(self):
        for n in list(self.procs):
            self.stop(n)
