import json
import os
import requests

TEST_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)))
MANIFEST = os.path.join(TEST_DIR, "../agnos.json")


from openpilot.common.test import OpenpilotTestCase
class TestAgnosUpdater(OpenpilotTestCase):

  def test_manifest(self):
    with open(MANIFEST) as f:
      m = json.load(f)

    for img in m:
      # follow redirects like agnos.py does (e.g. GitHub release downloads), and check for an xz stream
      r = requests.get(img['url'], timeout=10, stream=True, headers={'Range': 'bytes=0-5'})
      r.raise_for_status()
      assert r.raw.read(6) == b"\xfd7zXZ\x00"
      if not img['sparse']:
        assert img['hash'] == img['hash_raw']
