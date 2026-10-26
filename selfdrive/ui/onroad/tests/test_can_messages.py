import unittest

from openpilot.selfdrive.ui.onroad.can_messages import CanMessages


class TestCanMessages(unittest.TestCase):
  def test_latest_value_and_separate_buses(self):
    messages = CanMessages()
    messages.update(0, 0x123, b'\x00', 1.0)
    messages.update(1, 0x123, b'\xAB', 1.0)
    messages.update(0, 0x123, b'\xCD\xEF', 1.1)
    self.assertEqual(messages.lines(1.1), [' 0  00000123  00  CD EF', ' 1  00000123  00  AB'])

  def test_stale_messages_expire(self):
    messages = CanMessages(stale_after=2.0)
    messages.update(0, 1, b'\x01', 0.0)
    messages.update(0, 2, b'\x02', 1.0)
    self.assertEqual(messages.lines(2.0), [' 0  00000002  00  02'])
    self.assertEqual(messages.lines(3.0), [])

  def test_cache_evicts_least_recently_received(self):
    messages = CanMessages(max_messages=2)
    messages.update(0, 1, b'', 0.0)
    messages.update(0, 2, b'', 0.1)
    messages.update(0, 1, b'', 0.2)
    messages.update(0, 3, b'', 0.3)
    self.assertEqual(list(messages.messages), [(0, 1), (0, 3)])

  def test_can_fd_preserves_full_payload_and_extended_address(self):
    messages = CanMessages()
    data = bytes(range(64))
    messages.update(2, 0x18DA10F1, data, 0.0)
    lines = messages.lines(0.0)
    self.assertEqual(len(lines), 4)
    for index, line in enumerate(lines):
      self.assertTrue(line.startswith(f' 2  18DA10F1  {index * 16:02X}  '))
    self.assertEqual(bytes.fromhex(' '.join(line.split('  ', 3)[3] for line in lines)), data)

  def test_empty_payload_is_visible(self):
    messages = CanMessages()
    messages.update(0, 0, b'', 0.0)
    self.assertEqual(messages.lines(0.0), [' 0  00000000  00  (empty)'])

  def test_compact_rows_preserve_can_fd_payload(self):
    messages = CanMessages()
    data = bytes(range(64))
    messages.update(0, 0x123, data, 0.0)
    lines = messages.lines(0.0, bytes_per_line=8)
    self.assertEqual(len(lines), 8)
    self.assertEqual(bytes.fromhex(' '.join(line.split('  ', 3)[3] for line in lines)), data)


if __name__ == '__main__':
  unittest.main()
