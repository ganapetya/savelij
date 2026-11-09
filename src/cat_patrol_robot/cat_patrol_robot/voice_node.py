#!/usr/bin/env python3
# =============================================================================
# voice_node.py — plays a per-cat voice line when cat_detector_cpp recognizes
# a specific cat (white/brown), instead of the onboard buzzer beep.
# =============================================================================
#
# HOW IT WORKS:
#   1. On startup (run_ready_probe=true): wait until Pulse default sink matches
#      ready_sink_match (USB voice card by default), play ready-chime.mp3 through
#      the same ffmpeg|paplay path used for cat lines, then publish latched
#      /cat_patrol/audio_ready. patrol_manager gates driving on that topic
#      (fail-closed).
#   2. After the intro chime (or immediately if probe disabled): start looping
#      background music (cover1.mp3 by default) at a lower volume.
#   3. Subscribes to /cat_detector_cpp/detections (vision_msgs/Detection2DArray).
#   4. Each Detection2D carries up to two hypotheses in `results`:
#        results[0] = generic "cat" class (from YOLO)
#        results[1] = per-cat identity ("brown"/"white") + confidence,
#                     present only when the classifier fired.
#   5. On a confident identity (only after audio_ready): pause bg music, play
#      the next per-cat mp3 (jente-=white, arik-=brown, round-robin), then
#      resume bg music from where it left off.
#   6. A per-identity + global cooldown stops a lingering sighting from
#      re-triggering a new line every frame.
#
# WHY A SEPARATE NODE (not patrol_node)?
#   Keeping reactive/audio logic in its own Python node means it can be
#   started, stopped, and iterated on independently of the patrol FSM.
# =============================================================================

import fcntl
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray

PREFIX_BY_LABEL = {
    'white': 'jente-',
    'brown': 'arik-',
}

# Singleton lock: only one cat_voice may run at a time. Two instances would each
# react to the same detection and play overlapping mp3s (and each keeps its own
# cooldown, so a lingering cat retriggers constantly). A stray instance is easy
# to leave behind — e.g. `ros2 run ... &` then killing the wrapper orphans the
# node. An exclusive flock is race-free and self-healing: the kernel releases it
# automatically when the holder dies, so there is no stale-lock to clean up.
_LOCK_PATH = '/tmp/cat_voice.singleton.lock'


def acquire_singleton_lock():
    """Return the held lock file object, or None if another instance holds it.

    The returned object MUST stay referenced for the process lifetime — closing
    it (or letting it be garbage-collected) releases the lock.
    """
    fd = open(_LOCK_PATH, 'w')
    try:
        fcntl.flock(fd.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fd.close()
        return None
    fd.write(f'{os.getpid()}\n')
    fd.flush()
    return fd


def _get_default_sink():
    """Return PulseAudio default sink name, or '' if unavailable."""
    try:
        return subprocess.check_output(
            ['pactl', 'get-default-sink'], stderr=subprocess.DEVNULL, text=True
        ).strip()
    except (subprocess.CalledProcessError, FileNotFoundError, OSError):
        return ''


def _default_sink_matches(match_substring):
    """True when PulseAudio's default sink contains match_substring (case-insensitive).

    Empty match_substring means "any non-empty default sink" (useful for tests).
    """
    sink = _get_default_sink()
    if not sink:
        return False
    if not match_substring:
        return True
    return match_substring.lower() in sink.lower()


def _play_file_blocking(path, volume=1.0):
    """Play path on the current Pulse default sink; return True on success.

    Same path as cat lines: mp3 → ffmpeg → wav pipe → paplay. WAV uses paplay
    directly (no volume filter). volume is ffmpeg linear gain (1.0 = unity,
    1.1 = +10%). Waits for completion so callers can treat exit as readiness.
    """
    volume = max(0.0, min(4.0, float(volume)))

    if path.lower().endswith('.wav') and abs(volume - 1.0) < 1e-6:
        try:
            r = subprocess.run(
                ['paplay', path],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            return r.returncode == 0
        except OSError:
            return False

    try:
        cmd_ff = ['ffmpeg', '-loglevel', 'error', '-i', path]
        if abs(volume - 1.0) >= 1e-6:
            cmd_ff += ['-filter:a', f'volume={volume:.4f}']
        cmd_ff += ['-f', 'wav', '-']
        decode = subprocess.Popen(
            cmd_ff,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        play = subprocess.Popen(
            ['paplay'], stdin=decode.stdout,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if decode.stdout is not None:
            decode.stdout.close()
        play_rc = play.wait()
        decode.wait()
        return play_rc == 0
    except OSError:
        return False


def _kill_proc(proc):
    """Terminate a subprocess and its process group if possible."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except OSError:
            pass
    try:
        proc.wait(timeout=1.5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.kill()
            except OSError:
                pass
        try:
            proc.wait(timeout=1.0)
        except (subprocess.TimeoutExpired, OSError):
            pass


def _probe_duration_sec(path):
    """Return media duration in seconds via ffprobe, or None."""
    try:
        out = subprocess.check_output(
            [
                'ffprobe', '-v', 'error', '-show_entries', 'format=duration',
                '-of', 'default=noprint_wrappers=1:nokey=1', path,
            ],
            stderr=subprocess.DEVNULL, text=True, timeout=10,
        ).strip()
        return float(out)
    except (subprocess.CalledProcessError, FileNotFoundError, OSError,
            ValueError, subprocess.TimeoutExpired):
        return None


class BackgroundMusic:
    """Loop one mp3 on Pulse default sink; pause/resume around cat voice lines.

    Uses ffmpeg -stream_loop -1 so the file restarts from the beginning when it
    ends, with no gap in the outer player. Pause stops the player and remembers
    approximate offset so resume continues near the same place (ffmpeg -ss).
    """

    def __init__(self, path, volume=1.0, logger=None):
        self.path = path
        self.volume = max(0.0, min(4.0, float(volume)))
        self._log = logger
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._running = False
        self._want_play = False
        self._offset_sec = 0.0
        self._segment_start_mono = None
        self._duration = _probe_duration_sec(path)
        self._ffmpeg = None
        self._paplay = None
        self._thread = None

    def _info(self, msg):
        if self._log is not None:
            self._log.info(msg)

    def _warn(self, msg):
        if self._log is not None:
            self._log.warning(msg)

    def start(self):
        with self._cv:
            if not os.path.isfile(self.path):
                self._warn(f'bg music file missing: {self.path}')
                return
            self._running = True
            self._want_play = True
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(
                    target=self._loop, name='bg_music', daemon=True)
                self._thread.start()
                self._info(
                    f'bg music starting: {self.path} volume={self.volume:.2f} '
                    f'duration={self._duration if self._duration else "?"}s')
            self._cv.notify_all()

    def pause(self):
        """Stop playback and freeze position for later resume."""
        with self._cv:
            if not self._running:
                return
            self._want_play = False
            self._capture_offset_locked()
            self._stop_players_locked()
            self._cv.notify_all()

    def resume(self):
        with self._cv:
            if not self._running:
                return
            self._want_play = True
            self._cv.notify_all()

    def shutdown(self):
        with self._cv:
            self._running = False
            self._want_play = False
            self._stop_players_locked()
            self._cv.notify_all()
        t = self._thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=3.0)

    def _capture_offset_locked(self):
        if self._segment_start_mono is None:
            return
        elapsed = time.monotonic() - self._segment_start_mono
        pos = max(0.0, self._offset_sec + elapsed)
        # With infinite stream_loop, map wall-clock into one track length.
        if self._duration and self._duration > 0:
            pos = pos % self._duration
        self._offset_sec = pos
        self._segment_start_mono = None

    def _stop_players_locked(self):
        ff, pa = self._ffmpeg, self._paplay
        self._ffmpeg = None
        self._paplay = None
        self._segment_start_mono = None
        if pa is not None:
            _kill_proc(pa)
        if ff is not None:
            _kill_proc(ff)

    def _start_segment_locked(self):
        """Spawn ffmpeg|paplay from current offset. Caller holds lock.

        From the start of the file: infinite ffmpeg loop (-stream_loop -1) so
        the track restarts with no gap when it ends. From a mid-track resume:
        play once to the end; the outer _loop then restarts from 0 with infinite
        loop. (Combining -ss with -stream_loop would re-seek every wrap.)
        """
        self._stop_players_locked()
        ss = max(0.0, self._offset_sec)
        # Re-seek near end: wrap so we don't start a tiny stub.
        if self._duration and self._duration > 1.0 and ss >= self._duration - 0.25:
            ss = 0.0
            self._offset_sec = 0.0

        cmd_ff = ['ffmpeg', '-loglevel', 'error']
        if ss <= 0.05:
            ss = 0.0
            # Infinite loop from the beginning until pause/shutdown.
            cmd_ff += ['-stream_loop', '-1', '-i', self.path]
        else:
            # One pass from resume point to EOF; outer loop will wrap to start.
            cmd_ff += ['-ss', f'{ss:.3f}', '-i', self.path]
        cmd_ff += [
            '-filter:a', f'volume={self.volume:.4f}',
            '-f', 'wav', '-',
        ]
        try:
            # start_new_session → own process group for clean killpg
            decode = subprocess.Popen(
                cmd_ff,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                start_new_session=True)
            play = subprocess.Popen(
                ['paplay'], stdin=decode.stdout,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            if decode.stdout is not None:
                decode.stdout.close()
        except OSError as exc:
            self._warn(f'bg music spawn failed: {exc}')
            return False

        self._ffmpeg = decode
        self._paplay = play
        self._segment_start_mono = time.monotonic()
        self._offset_sec = ss
        return True

    def _loop(self):
        while True:
            with self._cv:
                while self._running and not self._want_play:
                    self._cv.wait(timeout=0.5)
                if not self._running:
                    self._stop_players_locked()
                    return
                if not self._start_segment_locked():
                    # Back off if spawn fails (Pulse down, etc.)
                    self._cv.wait(timeout=2.0)
                    continue
                pa = self._paplay
                ff = self._ffmpeg
                started_at_zero = self._offset_sec <= 0.05

            # Wait outside the lock so pause() can kill us.
            # Infinite loop from 0 blocks until pause/shutdown; mid-track play
            # ends at EOF (rc==0) and we wrap.
            rc = None
            if pa is not None:
                rc = pa.wait()
            if ff is not None:
                try:
                    ff.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    _kill_proc(ff)

            with self._cv:
                if not self._running:
                    self._stop_players_locked()
                    return
                if not self._want_play:
                    # pause() already captured offset
                    continue
                # Still want music: always continue from the start of the file
                # (natural end of mid-track resume, or unexpected player exit).
                self._offset_sec = 0.0
                self._segment_start_mono = None
                self._ffmpeg = None
                self._paplay = None
                if not started_at_zero or rc not in (0, None):
                    self._info(
                        f'bg music segment ended (rc={rc}); looping from start')
                self._cv.wait(timeout=0.05)


class VoiceNode(Node):
    """Plays a round-robin per-cat mp3 line when cat_detector_cpp recognizes a cat."""

    def __init__(self):
        super().__init__('cat_voice')

        self.declare_parameter('detections_topic', '/cat_detector_cpp/detections')
        self.declare_parameter('voices_dir', os.path.expanduser('~/cats/voices'))
        self.declare_parameter('min_identity_conf', 0.6)
        self.declare_parameter('cooldown_sec', 15.0)
        # Per-cat playlist start index (0-based). Useful for testing a specific line.
        self.declare_parameter('voice_index_start', 0)

        # --- Audio readiness gate (USB voice card end-to-end probe) ---
        self.declare_parameter('run_ready_probe', True)
        self.declare_parameter(
            'ready_sound_path',
            os.path.expanduser('~/cats/voices/ready-chime.mp3'))
        self.declare_parameter('audio_ready_topic', '/cat_patrol/audio_ready')
        self.declare_parameter('ready_timeout_sec', 90.0)
        self.declare_parameter('ready_poll_sec', 1.0)
        # Pulse default sink must contain this substring (case-insensitive).
        # Default matches JMTek / Solid State "USB PnP Audio Device" (0c76:1203).
        # Legacy Bluetooth: set to "bluez".
        self.declare_parameter('ready_sink_match', 'USB_PnP_Audio_Device')

        # --- Background music (after intro, between cat lines) ---
        self.declare_parameter('bg_music_enabled', True)
        self.declare_parameter(
            'bg_music_path',
            os.path.expanduser('~/cats/voices/cover1.mp3'))
        # Linear gain for ffmpeg volume filter (1.0 = unity / full file level).
        self.declare_parameter('bg_music_volume', 1.0)
        # Cat voice line gain (1.1 = +10% vs previous unity playback).
        self.declare_parameter('voice_volume', 1.1)

        topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.voices_dir = os.path.expanduser(
            self.get_parameter('voices_dir').get_parameter_value().string_value)
        self.min_conf = self.get_parameter('min_identity_conf').get_parameter_value().double_value
        self.cooldown_sec = self.get_parameter('cooldown_sec').get_parameter_value().double_value
        voice_index_start = int(
            self.get_parameter('voice_index_start').get_parameter_value().integer_value)

        self.ready_sound_path = os.path.expanduser(
            self.get_parameter('ready_sound_path').get_parameter_value().string_value)
        self.audio_ready_topic = (
            self.get_parameter('audio_ready_topic').get_parameter_value().string_value)
        self.ready_timeout_sec = (
            self.get_parameter('ready_timeout_sec').get_parameter_value().double_value)
        self.ready_poll_sec = (
            self.get_parameter('ready_poll_sec').get_parameter_value().double_value)
        self.ready_sink_match = (
            self.get_parameter('ready_sink_match').get_parameter_value().string_value)
        run_ready_probe = (
            self.get_parameter('run_ready_probe').get_parameter_value().bool_value)

        self.bg_music_enabled = (
            self.get_parameter('bg_music_enabled').get_parameter_value().bool_value)
        self.bg_music_path = os.path.expanduser(
            self.get_parameter('bg_music_path').get_parameter_value().string_value)
        self.bg_music_volume = (
            self.get_parameter('bg_music_volume').get_parameter_value().double_value)
        self.voice_volume = (
            self.get_parameter('voice_volume').get_parameter_value().double_value)

        self._files_by_label = self._index_voice_files()
        # Round-robin next index per identity (white/brown). Sorted file lists
        # so order is stable: jente-0, jente-1, … then wrap.
        self._next_index = {
            label: (voice_index_start % len(files) if files else 0)
            for label, files in self._files_by_label.items()
        }
        # Per-label cooldown (legacy) PLUS a global any-cat cooldown so a
        # white→brown flip within one second cannot play two lines.
        self._last_played = {}  # label -> monotonic timestamp of last playback
        self._last_played_any = 0.0  # monotonic of last playback of ANY cat
        self._audio_ready = False
        self._ready_lock = threading.Lock()
        self._voice_busy = False  # True while a cat line is playing
        self._voice_lock = threading.Lock()

        self._bg = None
        if self.bg_music_enabled:
            self._bg = BackgroundMusic(
                self.bg_music_path, volume=self.bg_music_volume, logger=self.get_logger())

        # Latched (transient_local) so patrol_manager can join late and still
        # see the last ready=true without a rediscovery race.
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.ready_pub = self.create_publisher(Bool, self.audio_ready_topic, ready_qos)

        self.sub = self.create_subscription(Detection2DArray, topic, self._on_detections, 10)
        playlist_summary = {
            k: f'{len(v)} files, next_idx={self._next_index[k]}'
            for k, v in self._files_by_label.items()
        }
        self.get_logger().info(
            f'cat_voice started. topic={topic} voices_dir={self.voices_dir} '
            f'playlist={playlist_summary} mode=round-robin '
            f'run_ready_probe={run_ready_probe} ready_sound={self.ready_sound_path} '
            f'ready_sink_match={self.ready_sink_match!r} '
            f'bg_music_enabled={self.bg_music_enabled} bg={self.bg_music_path} '
            f'bg_vol={self.bg_music_volume:.2f} voice_vol={self.voice_volume:.2f}')

        if run_ready_probe:
            self._probe_thread = threading.Thread(
                target=self._ready_probe_worker, name='audio_ready_probe', daemon=True)
            self._probe_thread.start()
        else:
            self.get_logger().warn(
                'run_ready_probe=false — marking audio ready without sink probe')
            self._mark_audio_ready()

    def destroy_node(self):
        if self._bg is not None:
            self._bg.shutdown()
        super().destroy_node()

    def _index_voice_files(self):
        files_by_label = {label: [] for label in PREFIX_BY_LABEL}
        if not os.path.isdir(self.voices_dir):
            self.get_logger().warning(f'voices_dir does not exist: {self.voices_dir}')
            return files_by_label
        for fn in sorted(os.listdir(self.voices_dir)):
            if not fn.lower().endswith('.mp3'):
                continue
            for label, prefix in PREFIX_BY_LABEL.items():
                if fn.startswith(prefix):
                    files_by_label[label].append(os.path.join(self.voices_dir, fn))
        for label, files in files_by_label.items():
            if not files:
                self.get_logger().warning(
                    f'No mp3 files matching "{PREFIX_BY_LABEL[label]}*" in {self.voices_dir}')
        return files_by_label

    def _mark_audio_ready(self):
        with self._ready_lock:
            self._audio_ready = True
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)
        self.get_logger().info(
            f'Audio READY published on {self.audio_ready_topic} (latched)')
        # Intro done — start looping cover music under cat lines.
        if self._bg is not None:
            self._bg.start()

    def _ready_probe_worker(self):
        """Wait for matching default sink, play ready chime, publish audio_ready.

        Fail-closed: on timeout or play failure we never publish true, so
        patrol_manager (require_audio_ready) will not leave WAKING to drive.
        connect_usb_speaker.sh may set the sink early; chime here is the
        deterministic end-to-end confirmation on the voice_node path.
        """
        match = self.ready_sink_match
        deadline = time.monotonic() + self.ready_timeout_sec
        self.get_logger().info(
            f'Audio ready probe: waiting up to {self.ready_timeout_sec:.0f}s for '
            f'default sink matching {match!r}, then play {self.ready_sound_path}')

        while time.monotonic() < deadline and rclpy.ok():
            if _default_sink_matches(match):
                break
            time.sleep(self.ready_poll_sec)
        else:
            sink = _get_default_sink() or '(none)'
            self.get_logger().error(
                f'Audio ready probe FAILED: Pulse default sink never matched '
                f'{match!r} within {self.ready_timeout_sec:.0f}s '
                f'(current={sink!r}). Cat voice suppressed; '
                'patrol will not drive if require_audio_ready=true.')
            return

        if not os.path.isfile(self.ready_sound_path):
            self.get_logger().error(
                f'Audio ready probe FAILED: ready sound missing: {self.ready_sound_path}')
            return

        sink = _get_default_sink()
        self.get_logger().info(
            f'Sink OK ({sink}) — playing ready chime: {self.ready_sound_path}')
        if not _play_file_blocking(self.ready_sound_path):
            self.get_logger().error(
                'Audio ready probe FAILED: paplay/ffmpeg play did not succeed')
            return

        if not _default_sink_matches(match):
            sink = _get_default_sink() or '(none)'
            self.get_logger().error(
                f'Audio ready probe FAILED: default sink left match {match!r} '
                f'during chime (current={sink!r})')
            return

        self._mark_audio_ready()

    def _on_detections(self, msg: Detection2DArray):
        with self._ready_lock:
            ready = self._audio_ready
        if not ready:
            return  # audio not proven yet — do not play into a missing/wrong sink

        # Best (highest-confidence) identity across all boxes in this frame.
        best_label, best_conf = None, -1.0
        for det in msg.detections:
            if len(det.results) < 2:
                continue  # identification didn't fire for this box
            id_hyp = det.results[1].hypothesis
            if id_hyp.class_id in PREFIX_BY_LABEL and id_hyp.score > best_conf:
                best_label, best_conf = id_hyp.class_id, id_hyp.score

        if best_label is None or best_conf < self.min_conf:
            return

        now = time.monotonic()
        # Global gate first: any recent play blocks ALL identities.
        if now - self._last_played_any < self.cooldown_sec:
            return
        last = self._last_played.get(best_label, 0.0)
        if now - last < self.cooldown_sec:
            return

        files = self._files_by_label.get(best_label, [])
        if not files:
            return

        with self._voice_lock:
            if self._voice_busy:
                return
            self._voice_busy = True

        self._last_played[best_label] = now
        self._last_played_any = now
        path, idx, n = self._next_voice_for(best_label)
        self.get_logger().info(
            f'Recognized {best_label} ({best_conf:.2f}) -> '
            f'playing [{idx + 1}/{n}] {os.path.basename(path)}')
        self._play_voice_line_async(path)

    def _next_voice_for(self, label):
        """Return (path, index_used, n_files) and advance that cat's counter."""
        files = self._files_by_label[label]
        n = len(files)
        idx = self._next_index.get(label, 0) % n
        path = files[idx]
        self._next_index[label] = (idx + 1) % n
        return path, idx, n

    def _play_voice_line_async(self, path):
        """Pause bg music, play cat line at voice_volume, resume bg music."""
        vol = self.voice_volume

        def worker():
            try:
                if self._bg is not None:
                    self._bg.pause()
                _play_file_blocking(path, volume=vol)
            finally:
                if self._bg is not None:
                    self._bg.resume()
                with self._voice_lock:
                    self._voice_busy = False

        threading.Thread(target=worker, name='cat_voice_line', daemon=True).start()


def main():
    rclpy.init()
    logger = rclpy.logging.get_logger('cat_voice')

    # Acquire the singleton lock BEFORE building the node, so a duplicate never
    # even creates its detections subscription (no window to play a stray line).
    lock = acquire_singleton_lock()
    if lock is None:
        logger.error(
            f'another cat_voice instance already holds {_LOCK_PATH}; exiting to '
            f'avoid overlapping playback')
        rclpy.shutdown()
        return

    node = VoiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()  # release the singleton lock


if __name__ == '__main__':
    main()
