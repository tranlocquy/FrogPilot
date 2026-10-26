from collections import OrderedDict


class CanMessages:
  """Bounded latest-value cache, keyed by CAN bus and address."""

  def __init__(self, max_messages: int = 512, stale_after: float = 2.0):
    self.max_messages = max_messages
    self.stale_after = stale_after
    self.messages: OrderedDict[tuple[int, int], tuple[bytes, float]] = OrderedDict()

  def update(self, bus: int, address: int, data: bytes, now: float) -> None:
    key = (bus, address)
    self.messages[key] = (data, now)
    self.messages.move_to_end(key)
    while len(self.messages) > self.max_messages:
      self.messages.popitem(last=False)

  def lines(self, now: float, bytes_per_line: int = 16) -> list[str]:
    for key, (_, received) in list(self.messages.items()):
      if now - received >= self.stale_after:
        del self.messages[key]

    lines = []
    for (bus, address), (data, _) in sorted(self.messages.items()):
      # Include every byte of CAN FD payloads, with a byte offset on each line.
      for offset in range(0, max(1, len(data)), bytes_per_line):
        payload = data[offset:offset + bytes_per_line].hex(' ').upper() or '(empty)'
        lines.append(f'{bus:2d}  {address:08X}  {offset:02X}  {payload}')
    return lines
