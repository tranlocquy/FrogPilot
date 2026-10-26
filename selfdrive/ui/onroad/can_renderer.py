import time
import pyray as rl

from cereal import messaging
from openpilot.selfdrive.ui.onroad.can_messages import CanMessages
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

REFRESH_INTERVAL = 0.2
PAGE_INTERVAL = 4.0
MAX_PACKETS_PER_FRAME = 64
LINES_PER_PAGE = 8


class CanRenderer(Widget):
  """Read-only raw CAN overlay. Never publishes or transmits CAN frames."""

  def __init__(self):
    super().__init__()
    self.set_enabled(False)
    self._font = gui_app.font(FontWeight.NORMAL)
    self._socket = None
    self._messages = CanMessages()
    self._lines: list[str] = []
    self._last_render = 0.0
    self._last_refresh = 0.0
    self._page_start = 0.0

  def _render(self, rect: rl.Rectangle) -> None:
    now = time.monotonic()
    compact = rect.height < 600
    lines_per_page = 4 if compact else LINES_PER_PAGE
    if not ui_state.started:
      self._socket = None
      return

    if self._socket is None or now - self._last_render > 1.0:
      self._socket = messaging.sub_sock('can')
      self._messages = CanMessages()
      self._lines = []
      self._last_refresh = 0.0
      self._page_start = now
    self._last_render = now

    # Bound the work per UI frame and never wait for traffic.
    for _ in range(MAX_PACKETS_PER_FRAME):
      packet = messaging.recv_one_or_none(self._socket)
      if packet is None:
        break
      for frame in packet.can:
        self._messages.update(frame.src, frame.address, bytes(frame.dat), now)

    if now - self._last_refresh >= REFRESH_INTERVAL:
      self._lines = self._messages.lines(now, bytes_per_line=8 if compact else 16)
      self._last_refresh = now

    pages = max(1, (len(self._lines) + lines_per_page - 1) // lines_per_page)
    page = int((now - self._page_start) / PAGE_INTERVAL) % pages
    lines = [f'RAW CAN {page + 1}/{pages} | BUS / ID / OFFSET / DATA']
    lines += self._lines[page * lines_per_page:(page + 1) * lines_per_page] or ['Waiting for CAN messages...']

    width = rect.width * (0.92 if compact else 0.62)
    padding = rect.height * 0.012
    font_size = max(14, rect.height * 0.028)
    widest = max(measure_text_cached(self._font, line, font_size).x for line in lines)
    if widest > width - 2 * padding:
      font_size *= (width - 2 * padding) / widest
    line_height = font_size * 1.35
    panel = rl.Rectangle(rect.x + rect.width - width - padding, rect.y + rect.height * 0.32,
                         width, line_height * len(lines) + 2 * padding)
    for index, line in enumerate(lines):
      rl.draw_text_ex(self._font, line, rl.Vector2(panel.x + padding, panel.y + padding + index * line_height),
                      font_size, 0, rl.WHITE)
