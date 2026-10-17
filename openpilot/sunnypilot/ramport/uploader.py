"""
Sends queued drive files to the upload service (contract sections 2, 3 and 4).

Rules:
- Only while the toggle "SunnyPilot/Ram Port data Upload" (setting EnableSunnylinkUploader) is on, and only on an
  unmetered network (setting NetworkMetered is false), so a cellular data plan is never used.
- Registration: when the setting RamPortDeviceId is empty, the device registers once. It sends its public key, its
  hardware serial number, its cellular-modem ID (IMEI, International Mobile Equipment Identity) and a register
  token. The token is a JSON Web Token (JWT, a small signed statement) signed with the device's own private key
  using RS256 (an RSA signature over a SHA-256 hash), the same key and method sunnylink and comma use. The service
  keeps only keyed hashes of the serial number and modem ID and answers with a random device ID.
- Upload: each file goes with a JWT signature that names the device ID, the file name, the file's SHA-256 hash
  (a fingerprint of its exact bytes), its size and a fresh random nonce (a number used once).
- Reactions to the service's answer:
    200, 201, 413, 422 -> done, delete the local file
    401, 403           -> register again, then retry
    409                -> retry with a new nonce
    429, 5xx, timeouts and network errors -> keep the file, wait 1 minute, doubling up to 6 hours
  Any other answer is treated like 5xx (keep the file, wait and retry).
"""
import hashlib
import os
import re
import secrets
import time
from dataclasses import dataclass
from enum import Enum

import jwt
import requests

from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.ramport.drive_format import FILE_NAME_RE, MAX_FILE_BYTES, wall_time
from openpilot.sunnypilot.ramport.drive_writer import DEFAULT_QUEUE_LIMIT_BYTES, enforce_queue_limit, queued_files

# PLACEHOLDER: the real service address is not known yet. Replace it once the Azure Function is deployed.
DEFAULT_API_HOST = "https://ramport-upload-relay.azurewebsites.net"
API_HOST_ENV = "RAMPORT_API_HOST"  # environment variable that overrides the address, for tests

REGISTER_PATH = "/api/register"
UPLOAD_PATH = "/api/upload"
JWT_ALGORITHM = "RS256"
REGISTER_TOKEN_LIFETIME_S = 30 * 60   # contract: at most 1 hour
UPLOAD_TOKEN_LIFETIME_S = 5 * 60      # contract: at most 10 minutes
NONCE_TRIES = 3

BACKOFF_START_S = 60.0
BACKOFF_MAX_S = 6 * 3600.0
IDLE_S = 10.0

DEVICE_ID_RE = re.compile(r"^[0-9a-f]{16}$")
USER_AGENT = "ramportd/1"


def api_host() -> str:
  return os.getenv(API_HOST_ENV, DEFAULT_API_HOST).rstrip("/")


@dataclass
class DeviceKey:
  private_key: str
  public_key_pem: str
  algorithm: str = JWT_ALGORITHM


def public_key_as_pem(public_key: str) -> str:
  """The device public key in PEM form (the text form that starts with -----BEGIN PUBLIC KEY-----).
  A key stored in OpenSSH form (starting with ssh-rsa) is converted."""
  text = public_key.strip()
  if text.startswith("-----BEGIN"):
    return text + "\n"
  from cryptography.hazmat.primitives import serialization
  key = serialization.load_ssh_public_key(text.encode())
  return key.public_bytes(serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo).decode()


def key_from_private_pem(private_pem: str) -> DeviceKey:
  """A DeviceKey from a private key alone (the public key is derived). Used by the command-line tool and tests."""
  from cryptography.hazmat.primitives import serialization
  key = serialization.load_pem_private_key(private_pem.encode(), password=None)
  public = key.public_key().public_bytes(serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo).decode()
  return DeviceKey(private_pem, public)


def load_device_key() -> DeviceKey | None:
  """The device's own key pair, found the same way sunnylink finds it (BaseApi.get_key_pair)."""
  from openpilot.common.api.base import BaseApi
  algorithm, private_key, public_key = BaseApi.get_key_pair()
  if not private_key or not public_key:
    return None
  if algorithm != JWT_ALGORITHM:
    cloudlog.error(f"ramport: device key uses {algorithm}, the upload service needs {JWT_ALGORITHM}")
    return None
  return DeviceKey(private_key, public_key_as_pem(public_key))


def default_hardware_ids(params) -> tuple[str, str]:
  """(hardware serial number, cellular-modem ID), read the same way sunnylink reads them."""
  from openpilot.common.hardware import HARDWARE
  serial = params.get("HardwareSerial") or ""
  if not serial:
    try:
      serial = HARDWARE.get_serial() or ""
    except Exception:
      serial = ""
  try:
    imei = HARDWARE.get_imei() or ""
  except Exception:
    imei = ""
  return str(serial), str(imei)


def make_register_token(key: DeviceKey, now: float | None = None) -> str:
  iat = int(wall_time() if now is None else now)
  token = jwt.encode({"register": True, "iat": iat, "exp": iat + REGISTER_TOKEN_LIFETIME_S}, key.private_key, algorithm=key.algorithm)
  return token if isinstance(token, str) else token.decode()


def make_upload_signature(key: DeviceKey, device_id: str, file_name: str, body: bytes, now: float | None = None,
                          nonce: str | None = None) -> str:
  iat = int(wall_time() if now is None else now)
  claims = {
    "device_id": device_id,
    "file_name": file_name,
    "sha256": hashlib.sha256(body).hexdigest(),
    "size": len(body),
    "nonce": nonce or secrets.token_hex(16),  # 32 random hexadecimal characters
    "iat": iat,
    "exp": iat + UPLOAD_TOKEN_LIFETIME_S,
  }
  token = jwt.encode(claims, key.private_key, algorithm=key.algorithm)
  return token if isinstance(token, str) else token.decode()


class RamPortClient:
  """Talks to the upload service. It only sends; the service offers nothing to read."""

  def __init__(self, host: str, key: DeviceKey, session: requests.Session | None = None, timeout=(10, 120)):
    self.host = host.rstrip("/")
    self.key = key
    self.session = session or requests.Session()
    self.timeout = timeout

  def register(self, serial: str, imei: str) -> requests.Response:
    body = {"public_key": self.key.public_key_pem, "serial": serial, "imei": imei, "register_token": make_register_token(self.key)}
    return self.session.post(self.host + REGISTER_PATH, json=body, timeout=self.timeout, allow_redirects=False,
                             headers={"User-Agent": USER_AGENT})

  def upload(self, device_id: str, file_name: str, body: bytes) -> requests.Response:
    headers = {
      "Content-Type": "application/gzip",
      "X-RamPort-Device": device_id,
      "X-RamPort-Signature": make_upload_signature(self.key, device_id, file_name, body),
      "User-Agent": USER_AGENT,
    }
    return self.session.post(self.host + UPLOAD_PATH, data=body, headers=headers, timeout=self.timeout, allow_redirects=False)


class Reaction(Enum):
  DONE = "done"              # delete the local file
  REGISTER_AGAIN = "register again"
  NEW_NONCE = "new nonce"
  RETRY_LATER = "retry later"


def upload_reaction(status_code: int) -> Reaction:
  if status_code in (200, 201, 413, 422):
    return Reaction.DONE
  if status_code in (401, 403):
    return Reaction.REGISTER_AGAIN
  if status_code == 409:
    return Reaction.NEW_NONCE
  return Reaction.RETRY_LATER


def parse_device_id(response: requests.Response) -> str | None:
  try:
    device_id = str(response.json().get("device_id", ""))
  except (ValueError, AttributeError):
    return None
  return device_id if DEVICE_ID_RE.match(device_id) else None


class QueueUploader:
  """Works through the queue one step at a time. run_once() returns how many seconds to wait before the next step."""

  def __init__(self, root: str, params, host: str | None = None, key_loader=load_device_key, hardware_ids=None,
               session: requests.Session | None = None, clock=time.monotonic, queue_limit_bytes: int = DEFAULT_QUEUE_LIMIT_BYTES,
               time_valid=None):
    self.root = root
    self.params = params
    self.host = host or api_host()
    self.key_loader = key_loader
    self.hardware_ids = hardware_ids or (lambda: default_hardware_ids(self.params))
    self.session = session or requests.Session()
    self.clock = clock
    self.queue_limit_bytes = queue_limit_bytes
    if time_valid is None:
      from openpilot.common.time_helpers import system_time_valid
      time_valid = system_time_valid
    self.time_valid = time_valid
    self.failures = 0
    self.next_try = 0.0
    self.last_status: int | None = None
    self.last_reaction: Reaction | None = None

  # --- conditions ---

  def allowed(self, onroad: bool = False) -> tuple[bool, str]:
    if not self.params.get_bool("EnableSunnylinkUploader"):
      return False, "toggle off"
    if self.params.get_bool("NetworkMetered"):
      return False, "metered network"
    if onroad and not self.params.get_bool("OnroadUploads"):
      return False, "onroad, uploads wait until parked"
    if not self.time_valid():
      return False, "clock not set yet"
    return True, ""

  def backoff_wait(self) -> float:
    return min(BACKOFF_START_S * (2 ** max(self.failures - 1, 0)), BACKOFF_MAX_S)

  def _fail(self, reason: str) -> float:
    self.failures += 1
    wait = self.backoff_wait()
    self.next_try = self.clock() + wait
    cloudlog.warning(f"ramport: {reason}; next try in {wait:.0f} s")
    return wait

  def _succeed(self) -> None:
    self.failures = 0
    self.next_try = 0.0

  # --- service steps ---

  def _client(self) -> RamPortClient | None:
    key = self.key_loader()
    if key is None:
      cloudlog.error("ramport: no device key found")
      return None
    return RamPortClient(self.host, key, session=self.session)

  def register(self, client: RamPortClient) -> str | None:
    """Register this device. Stores and returns the device ID, or None."""
    serial, imei = self.hardware_ids()
    if not serial or not imei:
      cloudlog.warning("ramport: hardware serial number or modem ID not available yet, cannot register")
      return None
    resp = client.register(serial, imei)
    self.last_status = resp.status_code
    if resp.status_code in (200, 201):
      device_id = parse_device_id(resp)
      if device_id:
        self.params.put("RamPortDeviceId", device_id, block=True)
        cloudlog.info(f"ramport: registered, device ID {device_id}")
        return device_id
      cloudlog.error("ramport: registration answer had no valid device ID")
    elif resp.status_code == 409:
      cloudlog.error("ramport: registration refused, this device key is already in use")
    elif resp.status_code == 429:
      cloudlog.warning("ramport: registration refused for now, new-device limit reached")
    else:
      cloudlog.warning(f"ramport: registration failed with status {resp.status_code}")
    return None

  def upload_file(self, client: RamPortClient, device_id: str, path: str) -> Reaction:
    """Send one file, retrying with a new nonce on 409. The caller handles the reaction."""
    name = os.path.basename(path)
    with open(path, "rb") as f:
      body = f.read()
    if len(body) > MAX_FILE_BYTES:
      cloudlog.warning(f"ramport: {name} is larger than the service accepts, deleting it")
      self.last_status = 413
      return Reaction.DONE

    reaction = Reaction.RETRY_LATER
    for _ in range(NONCE_TRIES):
      resp = client.upload(device_id, name, body)  # every call signs with a fresh nonce
      self.last_status = resp.status_code
      reaction = upload_reaction(resp.status_code)
      cloudlog.info(f"ramport: upload {name}: status {resp.status_code}, {reaction.value}")
      if reaction != Reaction.NEW_NONCE:
        return reaction
    return Reaction.RETRY_LATER

  def run_once(self, onroad: bool = False) -> float:
    ok, _reason = self.allowed(onroad)
    if not ok:
      return IDLE_S

    enforce_queue_limit(self.root, self.queue_limit_bytes)
    now = self.clock()
    if now < self.next_try:
      return min(self.next_try - now, IDLE_S)

    files = [p for p in queued_files(self.root) if FILE_NAME_RE.match(os.path.basename(p))]
    if not files:
      return IDLE_S
    path = files[0]

    client = self._client()
    if client is None:
      return self._fail("no device key")

    try:
      device_id = self.params.get("RamPortDeviceId") or ""
      if not device_id:
        device_id = self.register(client) or ""
        if not device_id:
          return self._fail("registration did not succeed")

      reaction = self.upload_file(client, device_id, path)
      if reaction == Reaction.REGISTER_AGAIN:
        self.params.remove("RamPortDeviceId")
        device_id = self.register(client) or ""
        if not device_id:
          self.last_reaction = Reaction.REGISTER_AGAIN
          return self._fail("registering again did not succeed")
        reaction = self.upload_file(client, device_id, path)
        if reaction == Reaction.REGISTER_AGAIN:
          reaction = Reaction.RETRY_LATER  # still refused right after registering: wait instead of looping
    except requests.RequestException as e:
      self.last_reaction = Reaction.RETRY_LATER
      return self._fail(f"network error ({type(e).__name__})")

    self.last_reaction = reaction
    if reaction == Reaction.DONE:
      try:
        os.unlink(path)
      except FileNotFoundError:
        pass
      self._succeed()
      return 0.0
    return self._fail(f"upload of {os.path.basename(path)} not accepted (status {self.last_status})")


def upload_file_once(path: str, host: str, key: DeviceKey, params, hardware_ids, session=None) -> tuple[Reaction, int | None, str]:
  """Register if needed and send one file, following the same reactions, without waiting or deleting anything.
  Used by the command-line tool for the end-to-end test. Returns (reaction, last status code, device ID)."""
  up = QueueUploader(os.path.dirname(path), params, host=host, key_loader=lambda: key, hardware_ids=hardware_ids, session=session,
                     time_valid=lambda: True)
  client = RamPortClient(host, key, session=up.session)
  device_id = params.get("RamPortDeviceId") or ""
  if not device_id:
    device_id = up.register(client) or ""
    if not device_id:
      return Reaction.RETRY_LATER, up.last_status, ""
  reaction = up.upload_file(client, device_id, path)
  if reaction == Reaction.REGISTER_AGAIN:
    params.remove("RamPortDeviceId")
    device_id = up.register(client) or ""
    if device_id:
      reaction = up.upload_file(client, device_id, path)
  return reaction, up.last_status, device_id
