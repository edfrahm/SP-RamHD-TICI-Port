"""
Tests for the ramportd uploader against a fake local service (tests/fake_service.py) that follows the contract.
The signing key is a test RSA key made inside the test, never the device key. All drive files are synthetic.
"""
import gzip
import json
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import jwt
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa

from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.ramport.drive_format import MAX_FILE_BYTES
from openpilot.sunnypilot.ramport.drive_writer import DriveWriter, queue_dir, queued_files
from openpilot.sunnypilot.ramport.ramportd import _cli, synthetic_fields
from openpilot.sunnypilot.ramport.tests.fake_service import FakeService
from openpilot.sunnypilot.ramport.uploader import (BACKOFF_MAX_S, IDLE_S, QueueUploader, RamPortClient, Reaction, key_from_private_pem,
                                                   load_device_key, upload_reaction)

SERIAL = "a1b2c3d4e5"
IMEI = "490154203237518"  # a well-known example modem ID with a valid check digit, not a real device's
META = {"truck": "RAM_HD_5TH_GEN", "engine_part": "52370231AF", "transmission_part": "", "software": "2026.003.000 abcdef123"}


def new_test_key_pem() -> str:
  key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
  return key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
                           serialization.NoEncryption()).decode()


class FakeClock:
  def __init__(self):
    self.now = 1000.0

  def __call__(self):
    return self.now


def unverified(token: str) -> dict:
  return jwt.decode(token, options={"verify_signature": False})


class TestUploader(OpenpilotTestCase):
  @classmethod
  def setup_class(cls):
    cls.private_pem = new_test_key_pem()
    cls.other_private_pem = new_test_key_pem()

  def setUp(self):
    super().setUp()
    self.params = Params()
    self.params.put_bool("EnableSunnylinkUploader", True, block=True)
    self.params.put_bool("NetworkMetered", False, block=True)
    self.root = Paths.ramport_root()
    self.key = key_from_private_pem(self.private_pem)
    self.clock = FakeClock()
    self.service = FakeService().__enter__()
    self.addCleanup(self.service.__exit__)

  def make_file(self, minutes: float = 5.0) -> str:
    writer = DriveWriter(self.root, lambda: META)
    for i in range(int(minutes * 600)):
      writer.add_sample(100.0 + i / 10, 1_792_000_000.0 + i / 10, synthetic_fields(i))
    paths = writer.end_drive()
    assert len(paths) == 1
    return paths[0]

  def uploader(self, host: str | None = None, **kwargs) -> QueueUploader:
    kwargs.setdefault("key_loader", lambda: self.key)
    kwargs.setdefault("hardware_ids", lambda: (SERIAL, IMEI))
    return QueueUploader(self.root, self.params, host=host or self.service.url, clock=self.clock, time_valid=lambda: True, **kwargs)

  # --- the normal path ---

  def test_register_then_upload_is_stored(self):
    path = self.make_file()
    body = open(path, "rb").read()
    up = self.uploader()
    assert up.run_once() == 0.0
    assert not os.path.exists(path), "a stored file is deleted"

    device_id = self.params.get("RamPortDeviceId")
    assert device_id and device_id in self.service.devices
    stored = list(self.service.stored[device_id].values())
    assert stored == [body], "the service received the exact file bytes"

    reg = self.service.registrations()[0]["body"]
    assert reg["public_key"] == self.key.public_key_pem and reg["serial"] == SERIAL and reg["imei"] == IMEI
    claims = jwt.decode(reg["register_token"], self.key.public_key_pem, algorithms=["RS256"])
    assert claims["register"] is True and 0 < claims["exp"] - claims["iat"] <= 3600
    # the service keeps only keyed hashes of the serial number and modem ID
    assert SERIAL not in json.dumps(self.service.devices) and IMEI not in json.dumps(self.service.devices)

    upload = self.service.uploads()[0]
    assert upload["content_type"] == "application/gzip" and upload["device_id"] == device_id
    c = upload["claims"]
    assert c["device_id"] == device_id and c["file_name"] == os.path.basename(path) and c["size"] == len(body)
    assert len(c["nonce"]) == 32 and 0 < c["exp"] - c["iat"] <= 600
    assert set(c) == {"device_id", "file_name", "sha256", "size", "nonce", "iat", "exp"}

  def test_signature_from_another_key_is_refused(self):
    """The fake service checks each signature with the registered public key, so the passing uploads above
    prove the device signs correctly. A signature from a different key is refused (401)."""
    path = self.make_file()
    up = self.uploader()
    device_id = up.register(up._client())
    other = RamPortClient(self.service.url, key_from_private_pem(self.other_private_pem))
    resp = other.upload(device_id, os.path.basename(path), open(path, "rb").read())
    assert resp.status_code == 401
    assert upload_reaction(resp.status_code) == Reaction.REGISTER_AGAIN

  def test_lost_setting_recovers_the_same_id(self):
    up = self.uploader()
    first = up.register(up._client())
    self.params.remove("RamPortDeviceId")
    again = up.register(up._client())
    assert first == again and up.last_status == 200

  def test_upload_order_oldest_first(self):
    a = self.make_file()
    b = self.make_file()
    os.utime(a, (2000, 2000))
    os.utime(b, (1000, 1000))
    up = self.uploader()
    up.run_once()
    assert os.path.exists(a) and not os.path.exists(b)

  # --- answers that finish a file: 200, 201, 413, 422 ---

  def test_duplicate_200_deletes(self):
    path = self.make_file()
    copy = os.path.join(queue_dir(self.root), "ffffffffffffffff_p01.csv.gz")
    with open(copy, "wb") as f:
      f.write(open(path, "rb").read())
    os.utime(copy, (os.path.getmtime(path) + 10,) * 2)
    up = self.uploader()
    up.run_once()
    up.run_once()
    assert up.last_status == 200 and up.last_reaction == Reaction.DONE
    assert queued_files(self.root) == []

  def test_too_large_413_deletes(self):
    path = self.make_file()
    self.service.max_bytes = 100
    up = self.uploader()
    up.run_once()
    assert up.last_status == 413 and not os.path.exists(path)

  def test_too_large_locally_is_deleted_without_sending(self):
    os.makedirs(queue_dir(self.root), exist_ok=True)
    path = os.path.join(queue_dir(self.root), "0123456789abcdef_p01.csv.gz")
    with open(path, "wb") as f:
      f.write(os.urandom(MAX_FILE_BYTES + 1))
    self.params.put("RamPortDeviceId", "0123456789abcdef", block=True)
    up = self.uploader()
    up.run_once()
    assert not os.path.exists(path) and self.service.uploads() == []

  def test_bad_file_422_deletes(self):
    os.makedirs(queue_dir(self.root), exist_ok=True)
    path = os.path.join(queue_dir(self.root), "0123456789abcdef_p01.csv.gz")
    with open(path, "wb") as f:
      f.write(gzip.compress(b"#RAMPORT {}\nnot,the,columns\n1,2,3\n"))
    up = self.uploader()
    up.run_once()
    assert up.last_status == 422 and not os.path.exists(path) and self.service.failures == 1

  # --- 401 and 403: register again, then retry ---

  def test_403_unknown_device_registers_again(self):
    path = self.make_file()
    self.params.put("RamPortDeviceId", "0000000000000000", block=True)
    up = self.uploader()
    assert up.run_once() == 0.0
    new_id = self.params.get("RamPortDeviceId")
    assert new_id != "0000000000000000" and new_id in self.service.devices
    assert [u["device_id"] for u in self.service.uploads()] == ["0000000000000000", new_id]
    assert not os.path.exists(path)

  def test_401_registers_again_then_retries(self):
    path = self.make_file()
    up = self.uploader()
    up.register(up._client())
    self.service.forced_upload = [401]
    assert up.run_once() == 0.0
    assert len(self.service.registrations()) == 2 and len(self.service.uploads()) == 2
    assert not os.path.exists(path)

  def test_401_again_after_registering_waits(self):
    path = self.make_file()
    self.service.forced_upload = [401, 401]
    up = self.uploader()
    assert up.run_once() == 60.0
    assert os.path.exists(path) and up.last_reaction == Reaction.RETRY_LATER

  # --- 409: retry with a new nonce ---

  def test_409_retries_with_a_new_nonce(self):
    path = self.make_file()
    up = self.uploader()
    up.register(up._client())
    self.service.forced_upload = [409]
    assert up.run_once() == 0.0
    tokens = [u["token"] for u in self.service.uploads()]
    assert len(tokens) == 2 and unverified(tokens[0])["nonce"] != unverified(tokens[1])["nonce"]
    assert not os.path.exists(path)

  def test_409_every_time_waits(self):
    path = self.make_file()
    self.service.forced_upload = [409, 409, 409]
    up = self.uploader()
    assert up.run_once() == 60.0
    assert os.path.exists(path) and len(self.service.uploads()) == 3

  # --- 429, 5xx, timeouts and network errors: keep the file, wait 1 minute doubling to 6 hours ---

  def test_backoff_doubles_up_to_six_hours(self):
    path = self.make_file()
    self.service.forced_upload = [429] + [503] * 11 + [500]
    up = self.uploader()
    waits = []
    for _ in range(13):
      waits.append(up.run_once())
      sent = len(self.service.uploads())
      self.clock.now += waits[-1] / 2
      assert up.run_once() <= IDLE_S and len(self.service.uploads()) == sent, "nothing is sent while waiting"
      self.clock.now += waits[-1] / 2
    assert waits == [60.0, 120.0, 240.0, 480.0, 960.0, 1920.0, 3840.0, 7680.0, 15360.0, BACKOFF_MAX_S, BACKOFF_MAX_S,
                     BACKOFF_MAX_S, BACKOFF_MAX_S]
    assert BACKOFF_MAX_S == 6 * 3600 and os.path.exists(path)
    assert up.run_once() == 0.0 and not os.path.exists(path), "after the service recovers the file goes through"
    assert up.failures == 0

  def test_network_error_waits(self):
    path = self.make_file()
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    closed_port = s.getsockname()[1]
    s.close()
    up = self.uploader(host=f"http://127.0.0.1:{closed_port}")
    assert up.run_once() == 60.0
    assert os.path.exists(path) and up.last_reaction == Reaction.RETRY_LATER

  def test_timeout_waits(self):
    path = self.make_file()

    class Slow(BaseHTTPRequestHandler):
      def log_message(self, *args):
        pass

      def do_POST(self):
        time.sleep(2.0)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Slow)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
      up = self.uploader(host=f"http://127.0.0.1:{server.server_address[1]}", timeout=(1, 0.3))
      assert up.run_once() == 60.0
      assert os.path.exists(path)
    finally:
      server.shutdown()
      server.server_close()

  # --- conditions: toggle, metered network, driving ---

  def test_metered_network_sends_nothing(self):
    path = self.make_file()
    self.params.put_bool("NetworkMetered", True, block=True)
    assert self.uploader().run_once() == IDLE_S
    assert self.service.requests == [] and os.path.exists(path)

  def test_toggle_off_sends_nothing(self):
    path = self.make_file()
    self.params.put_bool("EnableSunnylinkUploader", False, block=True)
    assert self.uploader().run_once() == IDLE_S
    assert self.service.requests == [] and os.path.exists(path)

  def test_while_driving_follows_onroad_uploads(self):
    path = self.make_file()
    self.params.put_bool("OnroadUploads", False, block=True)
    up = self.uploader()
    assert up.run_once(onroad=True) == IDLE_S and self.service.requests == []
    self.params.put_bool("OnroadUploads", True, block=True)
    assert up.run_once(onroad=True) == 0.0 and not os.path.exists(path)

  def test_clock_not_set_sends_nothing(self):
    self.make_file()
    up = QueueUploader(self.root, self.params, host=self.service.url, key_loader=lambda: self.key, hardware_ids=lambda: (SERIAL, IMEI),
                       clock=self.clock, time_valid=lambda: False)
    assert up.run_once() == IDLE_S and self.service.requests == []

  def test_no_files_sends_nothing(self):
    assert self.uploader().run_once() == IDLE_S and self.service.requests == []

  # --- registration answers ---

  def test_register_409_key_in_use(self):
    path = self.make_file()
    self.uploader(hardware_ids=lambda: ("other12345", "356938035643809")).register(self.uploader()._client())
    self.params.remove("RamPortDeviceId")
    up = self.uploader()
    assert up.run_once() == 60.0
    assert up.last_status == 409 and not self.params.get("RamPortDeviceId") and os.path.exists(path)

  def test_register_429_new_device_limit(self):
    path = self.make_file()
    self.service.forced_register = [429]
    up = self.uploader()
    assert up.run_once() == 60.0
    assert not self.params.get("RamPortDeviceId") and os.path.exists(path) and self.service.uploads() == []

  def test_no_modem_id_does_not_register(self):
    self.make_file()
    up = self.uploader(hardware_ids=lambda: (SERIAL, ""))
    assert up.run_once() == 60.0 and self.service.requests == []

  # --- queue size ---

  def test_queue_is_bounded(self):
    a = self.make_file()
    b = self.make_file()
    os.utime(a, (1000, 1000))
    os.utime(b, (2000, 2000))
    self.service.forced_upload = [503]
    up = self.uploader(queue_limit_bytes=os.path.getsize(b) + 10)
    up.run_once()
    assert not os.path.exists(a) and os.path.exists(b), "the oldest file goes first"

  # --- the device key, found the same way sunnylink finds it ---

  def test_device_key_loaded_like_sunnylink(self):
    key_dir = os.path.join(Paths.persist_root(), "comma")
    os.makedirs(key_dir, exist_ok=True)
    with open(os.path.join(key_dir, "id_rsa"), "w") as f:
      f.write(self.private_pem)
    with open(os.path.join(key_dir, "id_rsa.pub"), "w") as f:
      f.write(self.key.public_key_pem)
    loaded = load_device_key()
    assert loaded is not None and loaded.private_key == self.private_pem and loaded.public_key_pem.strip() == self.key.public_key_pem.strip()
    assert loaded.algorithm == "RS256"

    # a public key stored in OpenSSH form is sent as PEM
    ssh = serialization.load_pem_public_key(self.key.public_key_pem.encode()).public_bytes(serialization.Encoding.OpenSSH,
                                                                                           serialization.PublicFormat.OpenSSH)
    with open(os.path.join(key_dir, "id_rsa.pub"), "wb") as f:
      f.write(ssh)
    loaded = load_device_key()
    assert loaded is not None and loaded.public_key_pem.strip() == self.key.public_key_pem.strip()

  # --- the command-line tool used for the end-to-end test ---

  def test_cli_upload_file(self):
    path = self.make_file()
    key_path = os.path.join(Paths.comma_home(), "test_key.pem")
    with open(key_path, "w") as f:
      f.write(self.private_pem)
    rc = _cli(["--upload-file", path, "--url", self.service.url, "--key", key_path, "--serial", SERIAL, "--imei", IMEI])
    assert rc == 0
    assert os.path.exists(path), "the tool leaves the file in place"
    device_id = self.params.get("RamPortDeviceId")
    assert len(self.service.stored[device_id]) == 1
    # sending it again is a duplicate, still done
    assert _cli(["--upload-file", path, "--url", self.service.url, "--key", key_path, "--serial", SERIAL, "--imei", IMEI]) == 0
