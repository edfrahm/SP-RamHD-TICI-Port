"""The RamPort ID row on the Device settings page (selfdrive/ui/sunnypilot/layouts/settings/device.py), built without a
display (edfrahm/SP-RamHD-TICI-Port#9).

A row that reads its value before the page has set up what it reads crashes the screen process, and on the comma 3 the
screen then stays on the boot logo (this happened with the Weight Estimate row on 2026-10-19). So the whole page is
built here with stand-in widgets that read their values the way the real ones do: a text row reads its title and value
as soon as it is created. Everything else about the page stays the page's own code.
"""
from unittest import mock

import openpilot.selfdrive.ui.layouts.settings.device as device_base
import openpilot.selfdrive.ui.sunnypilot.layouts.settings.device as device_sp
from openpilot.common.test import OpenpilotTestCase


def resolve(value):
  return value() if callable(value) else value


class FakeItem(mock.MagicMock):
  """Any row. A MagicMock so the page can call whatever it likes on it (set_visible, action_item.right_button...)."""


def text_item(title, value, description=None, callback=None, enabled=True):
  item = FakeItem()
  item.title, item.value_source = resolve(title), value
  item.value = resolve(value)  # the real TextAction measures its text when it is created
  return item


def any_item(*args, **kwargs):
  item = FakeItem()
  title = kwargs.get("title", args[0] if args else None)
  if isinstance(title, str) or callable(title):
    item.title = resolve(title)
  return item


class FakeParams:
  def __init__(self, values=None):
    self.values = values or {}

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))


def build(values):
  params = FakeParams(values)
  builders = {"text_item": text_item, "button_item": any_item, "dual_button_item": any_item}
  sp_builders = {name: any_item for name in ("option_item_sp", "multiple_button_item_sp", "button_item_sp", "dual_button_item_sp")
                 if hasattr(device_sp, name)}
  with mock.patch.multiple(device_base, Params=lambda: params, Scroller=mock.MagicMock(), ui_state=mock.MagicMock(),
                           **{k: v for k, v in builders.items() if hasattr(device_base, k)}), \
       mock.patch.multiple(device_sp, text_item=text_item, LineSeparator=mock.MagicMock(), ui_state=mock.MagicMock(), **sp_builders):
    page = device_sp.DeviceLayoutSP()
  return page, params


class TestDevicePage(OpenpilotTestCase):
  def test_page_builds_and_shows_not_registered(self):
    page, _ = build({})
    self.assertEqual(page._ramport_id_text(), "not registered")

  def test_page_builds_with_an_id(self):
    page, _ = build({"RamPortDeviceId": "806c533ad18d56ac"})
    self.assertEqual(page._ramport_id_text(), "806c533ad18d56ac")

  def test_the_row_was_created_and_read_during_construction(self):
    created = []

    def recording_text_item(title, value, description=None, callback=None, enabled=True):
      item = text_item(title, value, description, callback, enabled)
      created.append(item)
      return item

    params = FakeParams({"RamPortDeviceId": "abc123"})
    with mock.patch.multiple(device_base, Params=lambda: params, Scroller=mock.MagicMock(), ui_state=mock.MagicMock(),
                             text_item=recording_text_item, button_item=any_item, dual_button_item=any_item), \
         mock.patch.multiple(device_sp, text_item=recording_text_item, LineSeparator=mock.MagicMock(), ui_state=mock.MagicMock(),
                             **{name: any_item for name in ("option_item_sp", "multiple_button_item_sp", "button_item_sp",
                                                            "dual_button_item_sp") if hasattr(device_sp, name)}):
      device_sp.DeviceLayoutSP()
    ramport = [item for item in created if item.title == "RamPort ID"]
    self.assertEqual(len(ramport), 1)
    self.assertEqual(ramport[0].value, "abc123")

  def test_reads_the_setting_at_most_once_per_second(self):
    page, params = build({"RamPortDeviceId": "first"})
    self.assertEqual(page._ramport_id_text(), "first")
    params.values["RamPortDeviceId"] = "second"
    self.assertEqual(page._ramport_id_text(), "first")
    page._ramport_id_read_time -= device_sp.RAMPORT_ID_REFRESH_S
    self.assertEqual(page._ramport_id_text(), "second")
