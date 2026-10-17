"""
Writes one drive into drive files (contract sections 1 and 2).

How the privacy trimming works:
- Samples from the first 2 minutes after the drive starts are dropped.
- Every later sample first waits in memory. It is written only once it is more than 2 minutes older than the
  newest sample. When the drive ends, the samples still waiting (the last 2 minutes) are thrown away.
So the first and the last 2 minutes of a drive never reach the disk.

Each part (at most one hour, 36,000 rows) is first collected as plain text rows in the "building" folder, next to
a small JSON note with its drive key, part number and start date. When the part is finished it is compressed
into the queue folder with its metadata line. If the process stops in the middle of a drive, the next start
finishes those leftovers: the rows on disk already exclude the last 2 minutes before the stop.
"""
import json
import os
import secrets
from collections import deque
from collections.abc import Callable
from datetime import datetime, UTC

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.ramport.drive_format import (COLUMN_NAMES, MAX_PART_SECONDS, MAX_ROWS_PER_PART, TRIM_S, build_metadata,
                                                       format_number, part_file_name, write_part_file)

ROWS_SUFFIX = ".rows"
NOTE_SUFFIX = ".json"
FLUSH_INTERVAL_S = 10.0
DEFAULT_QUEUE_LIMIT_BYTES = 500 * 1024 * 1024


def queue_dir(root: str) -> str:
  return os.path.join(root, "queue")


def building_dir(root: str) -> str:
  return os.path.join(root, "building")


def queued_files(root: str) -> list[str]:
  """Finished drive files waiting to be sent, oldest first."""
  qdir = queue_dir(root)
  try:
    names = [n for n in os.listdir(qdir) if n.endswith(".csv.gz")]
  except FileNotFoundError:
    return []
  paths = [os.path.join(qdir, n) for n in names]
  return sorted(paths, key=lambda p: (_mtime(p), p))


def _mtime(path: str) -> float:
  try:
    return os.path.getmtime(path)
  except OSError:
    return 0.0


def enforce_queue_limit(root: str, limit_bytes: int = DEFAULT_QUEUE_LIMIT_BYTES) -> list[str]:
  """Keep the queue under limit_bytes by deleting the oldest files first. Returns the deleted paths."""
  files = queued_files(root)
  sizes = {p: (os.path.getsize(p) if os.path.exists(p) else 0) for p in files}
  total = sum(sizes.values())
  deleted = []
  for path in files:
    if total <= limit_bytes:
      break
    try:
      os.unlink(path)
      deleted.append(path)
      total -= sizes[path]
      cloudlog.warning(f"ramport: queue over {limit_bytes} bytes, deleted oldest file {os.path.basename(path)}")
    except OSError:
      cloudlog.exception(f"ramport: could not delete {path}")
  return deleted


def utc_date(wall_time: float) -> str:
  return datetime.fromtimestamp(wall_time, UTC).strftime("%Y-%m-%d")


class _Part:
  """The part currently being collected in the building folder."""

  def __init__(self, root: str, drive_key: str, number: int, start_mono: float, date: str):
    self.drive_key = drive_key
    self.number = number
    self.start_mono = start_mono
    self.date = date
    self.rows = 0
    self.last_t = -1.0
    base = os.path.join(building_dir(root), f"{drive_key}_p{number:02d}")
    self.rows_path = base + ROWS_SUFFIX
    self.note_path = base + NOTE_SUFFIX
    with open(self.note_path, "w") as f:
      json.dump({"drive_key": drive_key, "part": number, "date": date}, f)
    self.file = open(self.rows_path, "w", encoding="utf-8", newline="\n")
    self.last_flush = start_mono

  def full(self, mono: float) -> bool:
    return self.rows >= MAX_ROWS_PER_PART or (mono - self.start_mono) >= MAX_PART_SECONDS

  def append(self, mono: float, fields: list[str]) -> None:
    t = round(mono - self.start_mono, 3)
    if t <= self.last_t:
      return  # t_s must strictly increase
    self.file.write(format_number(t, 3) + "," + ",".join(fields) + "\n")
    self.last_t = t
    self.rows += 1
    if mono - self.last_flush > FLUSH_INTERVAL_S:
      self.file.flush()
      self.last_flush = mono

  def close(self) -> None:
    if not self.file.closed:
      self.file.close()

  def discard(self) -> None:
    self.close()
    for path in (self.rows_path, self.note_path):
      try:
        os.unlink(path)
      except FileNotFoundError:
        pass


def _finish(root: str, rows_path: str, note_path: str, drive_key: str, number: int, date: str, last_part: bool,
            static_meta: dict, queue_limit_bytes: int) -> str | None:
  """Compress a collected part into the queue folder. Returns the queued path, or None if it had no rows."""
  with open(rows_path, encoding="utf-8", newline="\n") as f:
    lines = [line for line in f if line.endswith("\n") and line.count(",") == len(COLUMN_NAMES) - 1]
  out_path = None
  if lines:
    os.makedirs(queue_dir(root), exist_ok=True)
    metadata = build_metadata(static_meta, drive_key, number, len(lines), last_part, date)
    out_path = os.path.join(queue_dir(root), part_file_name(drive_key, number))
    write_part_file(out_path, metadata, lines)
    cloudlog.info(f"ramport: queued {os.path.basename(out_path)} with {len(lines)} rows")
  for path in (rows_path, note_path):
    try:
      os.unlink(path)
    except FileNotFoundError:
      pass
  if out_path is not None:
    enforce_queue_limit(root, queue_limit_bytes)
  return out_path


def recover_leftovers(root: str, static_meta: dict, queue_limit_bytes: int = DEFAULT_QUEUE_LIMIT_BYTES) -> list[str]:
  """Finish parts left in the building folder by a process that stopped mid-drive. Each becomes the last part
  of its drive. Its rows already exclude the 2 minutes before the stop, because those were only in memory."""
  bdir = building_dir(root)
  if not os.path.isdir(bdir):
    return []
  queued = []
  for name in sorted(os.listdir(bdir)):
    if not name.endswith(NOTE_SUFFIX):
      continue
    note_path = os.path.join(bdir, name)
    rows_path = note_path[:-len(NOTE_SUFFIX)] + ROWS_SUFFIX
    try:
      with open(note_path) as f:
        note = json.load(f)
      if os.path.exists(rows_path):
        out = _finish(root, rows_path, note_path, str(note["drive_key"]), int(note["part"]), str(note["date"]), True,
                      static_meta, queue_limit_bytes)
        if out:
          queued.append(out)
      else:
        os.unlink(note_path)
    except Exception:
      cloudlog.exception(f"ramport: could not recover {name}, removing it")
      for path in (note_path, rows_path):
        try:
          os.unlink(path)
        except OSError:
          pass
  # anything else left in the building folder has no note and cannot be finished
  for name in os.listdir(bdir):
    try:
      os.unlink(os.path.join(bdir, name))
    except OSError:
      pass
  return queued


class DriveWriter:
  """Collects the samples of one drive and writes its parts.

  meta_provider returns the static metadata (truck, engine_part, transmission_part, software). It is asked
  only when a part is finished, by which time the car's parameters are known.
  """

  def __init__(self, root: str, meta_provider: Callable[[], dict], drive_key: str | None = None,
               queue_limit_bytes: int = DEFAULT_QUEUE_LIMIT_BYTES):
    self.root = root
    self.meta_provider = meta_provider
    self.drive_key = drive_key or secrets.token_hex(8)  # 16 random lowercase hexadecimal characters
    self.queue_limit_bytes = queue_limit_bytes
    self.start_mono: float | None = None
    self.held: deque[tuple[float, float, list[str]]] = deque()  # (monotonic time, wall time, fields)
    self.part: _Part | None = None
    self.full_part: _Part | None = None  # a full part waits here until we know whether another part follows
    self.next_part_number = 1
    self.queued: list[str] = []
    self.ended = False
    os.makedirs(building_dir(root), exist_ok=True)

  def add_sample(self, mono: float, wall: float, fields: list[str]) -> None:
    """mono: monotonic clock in seconds. wall: Unix time in seconds (only its UTC date is written)."""
    if self.ended:
      return
    if self.start_mono is None:
      self.start_mono = mono
    if mono - self.start_mono < TRIM_S:
      return  # the first 2 minutes are never written

    self.held.append((mono, wall, fields))
    while self.held and mono - self.held[0][0] > TRIM_S:
      self._write(*self.held.popleft())

  def _write(self, mono: float, wall: float, fields: list[str]) -> None:
    if self.part is not None and self.part.full(mono):
      self.part.close()
      self.full_part = self.part
      self.part = None

    if self.part is None:
      if self.full_part is not None:  # another part follows, so the full one is not the last
        self._finish_part(self.full_part, last_part=False)
        self.full_part = None
      self.part = _Part(self.root, self.drive_key, self.next_part_number, mono, utc_date(wall))
      self.next_part_number += 1

    self.part.append(mono, fields)

  def _finish_part(self, part: _Part, last_part: bool) -> None:
    part.close()
    out = _finish(self.root, part.rows_path, part.note_path, part.drive_key, part.number, part.date, last_part,
                  self.meta_provider(), self.queue_limit_bytes)
    if out:
      self.queued.append(out)

  def end_drive(self) -> list[str]:
    """The drive ended: throw away the last 2 minutes and finish the last part. Returns all queued paths."""
    if self.ended:
      return self.queued
    self.ended = True
    self.held.clear()
    if self.part is not None:
      if self.full_part is not None:
        self._finish_part(self.full_part, last_part=False)
        self.full_part = None
      self._finish_part(self.part, last_part=True)
      self.part = None
    elif self.full_part is not None:
      self._finish_part(self.full_part, last_part=True)
      self.full_part = None
    return self.queued

  def abandon(self) -> None:
    """Stop without writing anything more: the samples in memory and the unfinished part are thrown away.
    Used when the upload toggle is switched off during a drive."""
    self.ended = True
    self.held.clear()
    for part in (self.part, self.full_part):
      if part is not None:
        part.discard()
    self.part = None
    self.full_part = None
