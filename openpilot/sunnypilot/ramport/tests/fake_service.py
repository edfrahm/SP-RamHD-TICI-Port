"""
A small local stand-in for the upload service, for the tests only. It follows the contract
("RamPort shared data upload: file format and upload protocol, version 1", sections 3 and 4): registration with
token and plausibility checks, keyed hashes of the serial number and modem ID, and the upload checks in the
contract's order. It stores nothing outside memory.
"""
import gzip
import hashlib
import hmac
import json
import re
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import jwt

from openpilot.sunnypilot.ramport.drive_format import COLUMN_NAMES, LIMITS, MARKER, METADATA_KEYS

HASH_SECRET = b"test-only-secret"
MAX_BYTES = 4_000_000


def luhn_ok(digits: str) -> bool:
  total = 0
  for i, ch in enumerate(reversed(digits)):
    d = int(ch)
    if i % 2 == 1:
      d *= 2
      if d > 9:
        d -= 9
    total += d
  return total % 10 == 0


def keyed_hash(value: str) -> str:
  return hmac.new(HASH_SECRET, value.encode(), hashlib.sha256).hexdigest()


def check_drive_file(body: bytes) -> bool:
  """Structure and value checks (contract section 1). True when the file is acceptable."""
  try:
    lines = gzip.decompress(body).decode("utf-8").split("\n")
    if lines[-1] == "":
      lines.pop()
    if not lines[0].startswith(MARKER):
      return False
    meta = json.loads(lines[0][len(MARKER):])
    if sorted(meta) != sorted(METADATA_KEYS) or meta["format"] != "ramport-drive" or meta["version"] != 1:
      return False
    if lines[1] != ",".join(COLUMN_NAMES):
      return False
    rows = lines[2:]
    if meta["rows"] != len(rows) or not 0 < len(rows) <= 36000:
      return False
    last_t = -1.0
    for line in rows:
      fields = line.split(",")
      if len(fields) != len(COLUMN_NAMES):
        return False
      t = float(fields[0])
      if t <= last_t:
        return False
      last_t = t
      for name, text in zip(COLUMN_NAMES[1:], fields[1:], strict=True):
        if text == "":
          continue
        if "e" in text.lower() or not re.fullmatch(r"-?\d+(\.\d{1,4})?", text):
          return False
        if name in LIMITS and not LIMITS[name][0] <= float(text) <= LIMITS[name][1]:
          return False
    return True
  except Exception:
    return False


class FakeService:
  def __init__(self):
    self.devices: dict[str, dict] = {}          # device_id -> {"public_key", "serial", "imei"} (hashes only)
    self.nonces: dict[str, set[str]] = {}
    self.stored: dict[str, dict[str, bytes]] = {}  # device_id -> sha256 -> body
    self.failures = 0
    self.forced_upload: list[int] = []          # status codes to answer to the next uploads, before any check
    self.forced_register: list[int] = []
    self.requests: list[dict] = []              # what arrived, for the tests to inspect
    self.max_bytes = MAX_BYTES
    self.lock = threading.Lock()
    service = self

    class Handler(BaseHTTPRequestHandler):
      def log_message(self, *args):
        pass

      def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)
        with service.lock:
          if self.path == "/api/register":
            status, answer = service.register(body)
          elif self.path == "/api/upload":
            status, answer = service.upload(dict(self.headers), body)
          else:
            status, answer = 404, {"error": "not found"}
        data = json.dumps(answer).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
    self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

  def __enter__(self):
    self.thread.start()
    return self

  def __exit__(self, *args):
    self.server.shutdown()
    self.server.server_close()

  # --- registration (contract section 3) ---

  def register(self, raw: bytes):
    try:
      req = json.loads(raw)
    except ValueError:
      return 400, {"error": "bad json"}
    self.requests.append({"path": "register", "body": req})
    if self.forced_register:
      return self.forced_register.pop(0), {}
    public_key, serial, imei = req.get("public_key", ""), req.get("serial", ""), req.get("imei", "")
    try:
      claims = jwt.decode(req.get("register_token", ""), public_key, algorithms=["RS256"], options={"require": ["exp", "iat"]})
    except jwt.PyJWTError:
      return 401, {"error": "bad token"}
    if claims.get("register") is not True or claims["exp"] - claims["iat"] > 3600:
      return 401, {"error": "bad token"}
    if not re.fullmatch(r"[A-Za-z0-9]{6,32}", serial) or not (re.fullmatch(r"\d{15}", imei) and luhn_ok(imei)):
      return 422, {"error": "implausible"}
    serial_hash, imei_hash = keyed_hash(serial), keyed_hash(imei)
    for device_id, dev in self.devices.items():
      same_key = dev["public_key"] == public_key
      same_ids = dev["serial"] == serial_hash and dev["imei"] == imei_hash
      if same_key and same_ids:
        return 200, {"device_id": device_id}
      if same_key or dev["serial"] == serial_hash or dev["imei"] == imei_hash:
        return 409, {"error": "key already in use"}
    device_id = secrets.token_hex(8)
    self.devices[device_id] = {"public_key": public_key, "serial": serial_hash, "imei": imei_hash}
    return 201, {"device_id": device_id}

  # --- upload (contract section 4), checks in the contract's order ---

  def upload(self, headers: dict, body: bytes):
    device_id = headers.get("X-RamPort-Device", "")
    token = headers.get("X-RamPort-Signature", "")
    entry = {"path": "upload", "device_id": device_id, "token": token, "size": len(body), "content_type": headers.get("Content-Type")}
    self.requests.append(entry)
    if self.forced_upload:
      return self.forced_upload.pop(0), {}
    if len(body) > self.max_bytes:
      return 413, {"error": "too large"}
    dev = self.devices.get(device_id)
    if dev is None:
      return 403, {"error": "unknown device"}
    try:
      claims = jwt.decode(token, dev["public_key"], algorithms=["RS256"], options={"require": ["exp", "iat"]})
    except jwt.PyJWTError:
      return 401, {"error": "bad signature"}
    entry["claims"] = claims
    # jwt.decode already refused an expired signature
    if (claims.get("device_id") != device_id or claims["exp"] - claims["iat"] > 600 or
        claims.get("sha256") != hashlib.sha256(body).hexdigest() or claims.get("size") != len(body) or not claims.get("file_name")):
      return 401, {"error": "bad signature"}
    nonce = str(claims.get("nonce", ""))
    if not re.fullmatch(r"[0-9a-f]{32}", nonce) or nonce in self.nonces.setdefault(device_id, set()):
      return 409, {"error": "nonce seen"}
    self.nonces[device_id].add(nonce)
    stored = self.stored.setdefault(device_id, {})
    if claims["sha256"] in stored:
      return 200, {"status": "duplicate"}
    if not check_drive_file(body):
      self.failures += 1
      return 422, {"error": "bad file"}
    stored[claims["sha256"]] = body
    return 201, {"status": "stored"}

  def uploads(self) -> list[dict]:
    return [r for r in self.requests if r["path"] == "upload"]

  def registrations(self) -> list[dict]:
    return [r for r in self.requests if r["path"] == "register"]
