"""
Tests for what the settings screens show for the shared data upload:
- the renamed toggle, exactly "SunnyPilot/Ram Port data Upload" with no other text, on the big screen (comma 3)
  and the small screen;
- the RamPort ID line on the Device page, "not registered" when empty.

The screens need a graphics window, which a test machine does not have, so these tests read the screen code
itself (with Python's own parser) and run the one small function that picks the RamPort ID text.
"""
import ast
import os
import time

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase

TITLE = "SunnyPilot/Ram Port data Upload"
UI = os.path.join(BASEDIR, "openpilot", "selfdrive", "ui", "sunnypilot")
BIG_SUNNYLINK = os.path.join(UI, "layouts", "settings", "sunnylink.py")
SMALL_SUNNYLINK = os.path.join(UI, "mici", "layouts", "sunnylink.py")
DEVICE = os.path.join(UI, "layouts", "settings", "device.py")
OLD_TEXTS = ("Enable sunnylink uploader", "infrastructure test", "upload your driving data to sunnypilot servers",
             "We are just testing data volume", "sunnylink uploader")


def parse(path: str) -> ast.Module:
  with open(path) as f:
    return ast.parse(f.read())


def constant(tree: ast.Module, name: str):
  for node in tree.body:
    if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets):
      return ast.literal_eval(node.value)
  raise AssertionError(f"{name} not found")


def calls(tree: ast.Module, func_name: str) -> list[ast.Call]:
  return [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", getattr(n.func, "attr", None)) == func_name]


def keywords(call: ast.Call) -> dict[str, ast.expr]:
  return {k.arg: k.value for k in call.keywords if k.arg}


class TestUploadToggle(OpenpilotTestCase):
  def test_big_screen_toggle(self):
    tree = parse(BIG_SUNNYLINK)
    assert constant(tree, "UPLOAD_TOGGLE_TITLE") == TITLE
    toggles = [c for c in calls(tree, "toggle_item_sp") if
               isinstance(keywords(c).get("param"), ast.Constant) and keywords(c)["param"].value == "EnableSunnylinkUploader"]
    assert len(toggles) == 1
    kw = keywords(toggles[0])
    assert isinstance(kw["title"], ast.Name) and kw["title"].id == "UPLOAD_TOGGLE_TITLE", "shown exactly, not translated"
    assert "description" not in kw and len(toggles[0].args) == 0, "no other on-screen text"

  def test_small_screen_toggle(self):
    tree = parse(SMALL_SUNNYLINK)
    assert constant(tree, "UPLOAD_TOGGLE_TITLE") == TITLE
    toggles = [c for c in calls(tree, "BigToggle") if
               isinstance(keywords(c).get("toggle_callback"), ast.Attribute) and keywords(c)["toggle_callback"].attr == "_sunnylink_uploader_callback"]
    assert len(toggles) == 1
    kw = keywords(toggles[0])
    assert isinstance(kw["text"], ast.Name) and kw["text"].id == "UPLOAD_TOGGLE_TITLE"
    # the toggle still writes the same setting
    with open(SMALL_SUNNYLINK) as f:
      assert 'put_bool("EnableSunnylinkUploader", state)' in f.read()

  def test_old_texts_are_gone(self):
    for path in (BIG_SUNNYLINK, SMALL_SUNNYLINK):
      with open(path) as f:
        source = f.read()
      for text in OLD_TEXTS:
        assert text not in source, (path, text)

  def test_translation_files_do_not_carry_the_old_texts(self):
    folder = os.path.join(BASEDIR, "openpilot", "selfdrive", "ui", "translations")
    for name in os.listdir(folder):
      if name.endswith((".po", ".ts", ".json")):
        with open(os.path.join(folder, name), encoding="utf-8") as f:
          source = f.read()
        for text in OLD_TEXTS:
          assert text not in source, (name, text)


class TestDevicePage(OpenpilotTestCase):
  def test_ramport_id_line(self):
    tree = parse(DEVICE)
    lines = [c for c in calls(tree, "text_item") if len(c.args) == 2 and isinstance(c.args[1], ast.Attribute)
             and c.args[1].attr == "_ramport_id_text"]
    assert len(lines) == 1
    title = lines[0].args[0]
    assert isinstance(title, ast.Lambda) and ast.literal_eval(title.body.args[0]) == "RamPort ID"

  def _ramport_id_text(self):
    """Build the screen's _ramport_id_text method from its source, without the graphics libraries."""
    tree = parse(DEVICE)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "DeviceLayoutSP")
    func = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_ramport_id_text")
    module = ast.Module(body=[func], type_ignores=[])
    scope = {"time": time, "tr": lambda s: s, "RAMPORT_ID_REFRESH_S": constant(tree, "RAMPORT_ID_REFRESH_S")}
    exec(compile(module, DEVICE, "exec"), scope)
    return scope["_ramport_id_text"]

  def test_ramport_id_text(self):
    method = self._ramport_id_text()

    class FakeParams:
      value: str | None = None

      def get(self, key):
        assert key == "RamPortDeviceId"
        return self.value

    class Screen:
      _params = FakeParams()
      _ramport_id = ""
      _ramport_id_read_time = None

    screen = Screen()
    assert method(screen) == "not registered"
    screen._params.value = "0123456789abcdef"
    screen._ramport_id_read_time = None  # next read
    assert method(screen) == "0123456789abcdef"
    screen._params.value = ""
    assert method(screen) == "0123456789abcdef", "read at most once per second"
    screen._ramport_id_read_time -= 2.0
    assert method(screen) == "not registered"
