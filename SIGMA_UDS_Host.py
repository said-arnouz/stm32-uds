"""
SIGMA_UDS_GUI_PyQt5.py  –  UDS UART HOST  (PyQt5 GUI)
────────────────────────────────────────────────────────
• QTreeWidget  → fully dynamic / resizable columns
• Persistent background reader thread (captures every ECU frame)
• Real COM-port liveness check (ping STM32 on connect)
• Port selector with live ● indicator per port

Requirements:
    pip install pyserial PyQt5
Usage:
    python SIGMA_UDS_GUI_PyQt5.py
"""

import sys, time, threading
import serial, serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFrame, QLabel, QPushButton,
    QLineEdit, QComboBox, QVBoxLayout, QHBoxLayout,
    QStatusBar, QTreeWidget, QTreeWidgetItem, QHeaderView,
    QAbstractItemView, QStyledItemDelegate
)
from PyQt5.QtGui  import (
    QFont, QColor, QPalette, QPixmap, QBrush, QPainter, QPen, QTextDocument,QPainterPath
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QSize, QRectF

# ═══════════════════════════════════════════════════════════════════════════
# CONFIG
# ═══════════════════════════════════════════════════════════════════════════
FRAME_SIZE = 8
PAD_BYTE   = 0xAA

# ═══════════════════════════════════════════════════════════════════════════
# COLORS  (light professional theme)
# ═══════════════════════════════════════════════════════════════════════════
C = {
    "bg":            "#F5F5F5",
    "panel":         "#ECECEC",
    "header":        "#FFFFFF",
    "border":        "#D0D0D0",
    "accent_red":    "#C0392B",
    "accent_green":  "#00963D",
    "text":          "#111111",
    "text_dim":      "#6C757D",
    "row_even":      "#FFFFFF",
    "row_odd":       "#F4F4F4",
    "row_hover":     "#E3F0FF",
    "row_select":    "#D0E8FF",
    "input_bg":      "#FFFFFF",
    "btn_send":      "#00963D",
    "btn_send_fg":   "#FFFFFF",
    "btn_clear":     "#CCCCCC",
    "btn_clear_fg":  "#333333",
    # byte colours
    "pci":           "#C62828",
    "sid_req":       "#1565C0",
    "sid_resp":      "#E65100",
    "did":           "#00838F",
    "payload":       "#2E7D32",
    "padding":       "#9E9E9E",
    # CAN ID
    "can_client":    "#00963D",
    "can_ecu":       "#E65100",
    # table header
    "col_hdr_bg":    "#E0E0E0",
    "col_hdr_fg":    "#444444",
    "legend_bg":     "#F0F0F0",
}

BYTE_TAG_COLORS = {k: C[k] for k in ("pci","sid_req","sid_resp","did","payload","padding")}

# Column index constants
COL_TIME, COL_PROTO, COL_SVC, COL_CAN, COL_BYTES, COL_SENDER, COL_FRAME = range(7)

# ═══════════════════════════════════════════════════════════════════════════
# UDS METADATA
# ═══════════════════════════════════════════════════════════════════════════
SERVICE_NAMES = {
    0x10:"DiagnosticSessionControl", 0x11:"ECUReset",
    0x22:"ReadDataByIdentifier",     0x27:"SecurityAccess",
    0x2E:"WriteDataByIdentifier",    0x31:"RoutineControl",
    0x34:"RequestDownload",          0x36:"TransferData",
    0x37:"RequestTransferExit",
    0x50:"DiagnosticSessionControl", 0x51:"ECUReset",
    0x62:"ReadDataByIdentifier",     0x67:"SecurityAccess",
    0x6E:"WriteDataByIdentifier",    0x71:"RoutineControl",
    0x74:"RequestDownload",          0x76:"TransferData",
    0x77:"RequestTransferExit",
    0x7F:"NegativeResponse",
}
SESSION_NAMES  = {0x01:"Default Session",   0x02:"Programming Session", 0x03:"Extended Session"}
RESET_NAMES    = {0x01:"Hard Reset",        0x02:"Key Off/On Reset",    0x03:"Soft Reset"}
SECURITY_NAMES = {0x01:"Request Seed",      0x02:"Send Key"}
ROUTINE_NAMES  = {0x01:"Start Routine",     0x02:"Stop Routine",        0x03:"Request Results"}
NRC_NAMES = {
    0x10:"generalReject",            0x11:"serviceNotSupported",
    0x12:"subFunctionNotSupported",  0x13:"incorrectMessageLength",
    0x14:"responseTooLong",          0x22:"conditionsNotCorrect",
    0x24:"requestSequenceError",     0x31:"requestOutOfRange",
    0x33:"securityAccessDenied",     0x35:"invalidKey",
    0x36:"exceededNumberOfAttempts", 0x37:"requiredTimeDelayNotExpired",
    0x72:"generalProgrammingFailure",
}

# ═══════════════════════════════════════════════════════════════════════════
# UDS HELPERS
# ═══════════════════════════════════════════════════════════════════════════
def parse_input(s: str):
    s = s.strip().upper().replace(" ","")
    if s.startswith("0X"): s = s[2:]
    return [int(s[i:i+2],16) for i in range(0,len(s),2) if len(s[i:i+2])==2]

def build_frame(payload: list) -> bytes:
    f = [len(payload)] + payload
    while len(f) < FRAME_SIZE: f.append(PAD_BYTE)
    return bytes(f[:FRAME_SIZE])

def describe_frame(frame: bytes, sender: str):
    sid     = frame[1] if len(frame)>1 else 0
    sub     = frame[2] if len(frame)>2 else 0
    name    = SERVICE_NAMES.get(sid, f"Unknown(0x{sid:02X})")
    can     = "0x7E8" if sender=="ECU" else "0x7E0"
    is_resp = (0x40 <= sid < 0x80) or sid==0x7F

    if   sid==0x7F:
        nrc = frame[3] if len(frame)>3 else 0
        det = NRC_NAMES.get(nrc, f"0x{nrc:02X}")
    elif sid in (0x10,0x50): det = SESSION_NAMES.get(sub, f"0x{sub:02X}")
    elif sid in (0x11,0x51): det = RESET_NAMES.get(sub,   f"0x{sub:02X}")
    elif sid in (0x27,0x67): det = SECURITY_NAMES.get(sub,f"Sub 0x{sub:02X}")
    elif sid in (0x31,0x71): det = ROUTINE_NAMES.get(sub, f"Sub 0x{sub:02X}")
    elif sid in (0x22,0x62):
        did = (frame[2]<<8|frame[3]) if len(frame)>3 else 0
        det = f"Sub 0x{did:04X}"
    else: det = "—"

    pci_len = frame[0]
    colored = []
    for i,b in enumerate(frame):
        hx  = f"{b:02X}"
        pos = i-1
        if   i==0:                                   colored.append((hx,"pci"))
        elif b==PAD_BYTE and i>pci_len:              colored.append((hx,"padding"))
        elif pos==0:                                 colored.append((hx,"sid_resp" if is_resp else "sid_req"))
        elif sid in (0x22,0x62) and pos in (1,2):   colored.append((hx,"did"))
        elif sid==0x7F:
            colored.append((hx, "did" if pos==1 else "sid_resp" if pos==2 else "payload"))
        else:                                        colored.append((hx,"payload"))

    return name, det, can, colored, sender

def bytes_html(colored):
    return "&nbsp;".join(
        f'<span style="color:{BYTE_TAG_COLORS.get(t,C["text"])};font-weight:normal;">{h}</span>'
        for h,t in colored)

# ═══════════════════════════════════════════════════════════════════════════
# LOGO
# ═══════════════════════════════════════════════════════════════════════════
def make_logo(size=40) -> QPixmap:
    src = QPixmap(r"C:\Users\HP\Documents\work_space\UDS\images\logo.jpg")
    
    w, h = src.width(), src.height()
    side = min(w, h)
    src  = src.copy((w-side)//2, (h-side)//2, side, side)
    src  = src.scaled(size, size, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
    
    result = QPixmap(size, size)
    result.fill(Qt.transparent)
    
    p = QPainter(result)
    p.setRenderHint(QPainter.Antialiasing)
    path = QPainterPath()
    path.addEllipse(0, 0, size, size)
    p.setClipPath(path)
    p.drawPixmap(0, 0, src)
    p.end()
    
    return result
# ═══════════════════════════════════════════════════════════════════════════
# HTML DELEGATE  – renders colored bytes inside QTreeWidget cells
# ═══════════════════════════════════════════════════════════════════════════
class HtmlDelegate(QStyledItemDelegate):
    def paint(self, painter, option, index):
        bg = index.data(Qt.BackgroundRole)
        painter.fillRect(option.rect,
            bg if isinstance(bg, QBrush) else QBrush(QColor(C["row_even"])))
        html = index.data(Qt.UserRole)
        if not html:
            super().paint(painter, option, index); return
        doc = QTextDocument()
        doc.setDefaultFont(QFont("Consolas",9))
        doc.setHtml(f'<span style="font-family:Consolas;font-size:9pt;font-weight:normal;">{html}</span>')
        painter.save()
        y = option.rect.top() + (option.rect.height() - doc.size().height()) / 2
        painter.translate(option.rect.left()+6, y)
        doc.drawContents(painter, QRectF(0, 0, option.rect.width()-8, option.rect.height()))
        painter.restore()
        pen = QPen(QColor(C["border"]))
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawLine(option.rect.left(), option.rect.bottom(),
                        option.rect.right(), option.rect.bottom())
    def sizeHint(self, option, index): return QSize(200, 22)

# ═══════════════════════════════════════════════════════════════════════════
# BUTTON / LABEL FACTORIES
# ═══════════════════════════════════════════════════════════════════════════
def _btn(text, bg, fg="#FFF", mw=80, mh=30):
    b = QPushButton(text)
    f = QFont("Segoe UI",9); f.setBold(True); b.setFont(f)
    b.setStyleSheet(f"""
        QPushButton{{background:{bg};color:{fg};border:none;
            border-radius:4px;padding:5px 14px;}}
        QPushButton:hover{{background:{bg}CC;}}
        QPushButton:pressed{{background:{bg}88;}}""")
    b.setMinimumSize(mw, mh); b.setCursor(Qt.PointingHandCursor)
    return b

def _lbl(text, fg=None, bold=False, size=9, family="Segoe UI"):
    l = QLabel(text); f = QFont(family, size); f.setBold(bold); l.setFont(f)
    l.setStyleSheet(f"color:{fg or C['text_dim']};background:transparent;")
    return l

# ═══════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ═══════════════════════════════════════════════════════════════════════════
class SigmaUDSApp(QMainWindow):
    _row_sig    = pyqtSignal(float, str, str, str, list, str)
    _status_sig = pyqtSignal(str, bool)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UDS Simulator")
        self.resize(1280, 780); self.setMinimumSize(900, 600)
        self._ser          = None
        self._connected    = False
        self._start_time   = None
        self._row_count    = 0
        self._reader_stop  = threading.Event()

        self.setStyleSheet(f"QMainWindow{{background-color:{C['bg']};}}")

        self._row_sig.connect(self._add_row)
        self._status_sig.connect(self._set_status)
        self._build_ui()
        self._refresh_ports()

        # Poll queue + port list every 2s
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(2000)

    # ───────────────────────────────────────────────────────────────────────
    # BUILD UI
    # ───────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        root_w = QWidget(); root_w.setStyleSheet(f"background:{C['bg']};")
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w); root.setContentsMargins(0,0,0,0); root.setSpacing(0)

        # ── HEADER ─────────────────────────────────────────────────────────
        hdr = QFrame(); hdr.setFixedHeight(60)
        hdr.setStyleSheet(f"background:{C['header']};border:none;")
        hl = QHBoxLayout(hdr); hl.setContentsMargins(14,8,16,8); hl.setSpacing(8)

        logo = QLabel(); logo.setPixmap(make_logo(40)); logo.setFixedSize(40,40)
        logo.setStyleSheet("background:transparent;"); hl.addWidget(logo)

        t = QLabel("SIGMA Embedded"); tf = QFont("Segoe UI",15); tf.setBold(True)
        t.setFont(tf); t.setStyleSheet(f"color:{C['text']};background:transparent;")
        hl.addWidget(t); hl.addStretch()

        self._dot = QLabel("●")
        self._dot.setStyleSheet("color:#AAAAAA;background:transparent;font-size:13px;")
        hl.addWidget(self._dot)
        self._conn_lbl = QLabel("Not Connected")
        cf = QFont("Segoe UI",9); cf.setBold(True)
        self._conn_lbl.setFont(cf)
        self._conn_lbl.setStyleSheet(f"color:{C['text_dim']};background:transparent;")
        hl.addWidget(self._conn_lbl)
        root.addWidget(hdr)

        # ── CONNECTION BAR ─────────────────────────────────────────────────
        cb = QFrame()
        cb.setStyleSheet(f"background:{C['panel']};")
        cl = QHBoxLayout(cb); cl.setContentsMargins(12,5,12,5); cl.setSpacing(8)

        cbo_ss = f"""
            QComboBox{{background:{C['input_bg']};color:{C['text']};
                border:None;padding:3px 6px;font-family:Segoe UI;font-size:8pt;}}
            QComboBox QAbstractItemView{{background:{C['input_bg']};color:{C['text']};
                selection-background-color:{C['border']};}}"""

        cl.addWidget(_lbl("Port:"))
        self._port_combo = QComboBox(); 
        self._port_combo.setFixedWidth(100);
        self._port_combo.setStyleSheet(cbo_ss)
        cl.addWidget(self._port_combo)

        cl.addWidget(_lbl("Baud:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["9600","19200","38400","57600","115200","230400","460800","921600"])
        self._baud_combo.setCurrentText("115200")
        self._baud_combo.setFixedWidth(100); self._baud_combo.setStyleSheet(cbo_ss)
        cl.addWidget(self._baud_combo)

        self._conn_btn = _btn("Connect", C["btn_send"], C["btn_send_fg"], 110, 30)
        self._conn_btn.clicked.connect(self._toggle_connection)
        cl.addWidget(self._conn_btn)

        self._conn_status = _lbl("● Disconnected", fg=C["accent_red"])
        self._conn_status.setFont(QFont("Segoe UI", 8))
        cl.addWidget(self._conn_status)
        cl.addStretch()
        root.addWidget(cb)

        # ── INPUT BAR ──────────────────────────────────────────────────────
        ib = QFrame()
        ib.setStyleSheet(f"background:{C['panel']};border-bottom:1px solid {C['border']};")
        il = QHBoxLayout(ib); il.setContentsMargins(12,6,12,6); il.setSpacing(8)

        self._input = QLineEdit()
        ef = QFont("Consolas",9); ef.setBold(False); self._input.setFont(ef)
        self._input.setStyleSheet(f"""
            QLineEdit{{background:{C['input_bg']};color:{C['text']};
                border:1px solid {C['border']};border-radius:3px;padding:6px 10px;}}
            QLineEdit:focus{{border:1px solid {C['accent_green']};}}""")
        self._input.returnPressed.connect(self._send_request)
        il.addWidget(self._input, stretch=1)

        sb = _btn("Send Request", C["btn_send"], C["btn_send_fg"], 130, 32)
        sb.clicked.connect(self._send_request); il.addWidget(sb)

        clr = _btn("Clear", C["btn_clear"], C["btn_clear_fg"], 70, 32)
        clr.clicked.connect(self._clear_trace); il.addWidget(clr)
        root.addWidget(ib)

        # ── LEGEND BAR ─────────────────────────────────────────────────────
        leg = QFrame(); leg.setFixedHeight(30)
        leg.setStyleSheet(f"background:{C['legend_bg']};border-bottom:1px solid {C['border']};")
        ll = QHBoxLayout(leg); ll.setContentsMargins(12,0,12,0); ll.setSpacing(4)
        tw = _lbl("Trace window", fg=C["text"], bold=True); ll.addWidget(tw)
        ll.addStretch()
        for name, col in [("PCI",C["pci"]),("SID REQ",C["sid_req"]),("DID",C["did"]),
                           ("SID RESP",C["sid_resp"]),("PAYLOAD",C["payload"]),("PADDING",C["padding"])]:
            x = QLabel(f"■ {name}"); x.setFont(QFont("Segoe UI",8))
            x.setStyleSheet(f"color:{col};background:transparent;padding:0 6px;"); ll.addWidget(x)
        root.addWidget(leg)

        # ── TREE TABLE ─────────────────────────────────────────────────────
        self._tree = QTreeWidget()
        self._tree.setColumnCount(7)
        self._tree.setHeaderLabels([
            "Time","Protocol Service","Service",
            "CAN ID (HEX)","Data Bytes (HEX)","Sender","Frame Type"])
        self._tree.setRootIsDecorated(False)
        self._tree.setAlternatingRowColors(True)
        self._tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self._tree.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._tree.setUniformRowHeights(True)
        self._tree.setIndentation(0)
        self._tree.setItemDelegateForColumn(COL_BYTES, HtmlDelegate(self._tree))

        hh = self._tree.header()
        hh.setSectionResizeMode(COL_TIME,   QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(COL_PROTO,  QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(COL_SVC,    QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(COL_CAN,    QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(COL_BYTES,  QHeaderView.Stretch)
        hh.setSectionResizeMode(COL_SENDER, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(COL_FRAME,  QHeaderView.ResizeToContents)
        hh.setStretchLastSection(False)
        hh.setMinimumSectionSize(70)
        # sensible initial widths
        hh.resizeSection(COL_PROTO, 220)
        hh.resizeSection(COL_SVC,   180)

        self._tree.setStyleSheet(f"""
            QTreeWidget{{
                background:{C['bg']};alternate-background-color:{C['row_odd']};
                border:none;outline:none;
                font-family:Consolas;font-size:9pt;
                show-decoration-selected:0;}}
            QTreeWidget::item{{
                height:24px;
                border-bottom:1px solid {C['border']};
                padding:0 4px;}}
            QTreeWidget::item:hover{{background:{C['row_hover']};}}
            QTreeWidget::item:selected{{background:{C['row_select']};color:{C['text']};}}
            QHeaderView::section{{
                background:{C['col_hdr_bg']};color:{C['col_hdr_fg']};
                font-family:Segoe UI;font-size:8pt;font-weight:bold;
                padding:4px 8px;
                border:none;
                border-right:1px solid {C['border']};
                border-bottom:2px solid {C['border']};}}
            QHeaderView::section:last{{border-right:none;}}
            QScrollBar:vertical{{background:{C['panel']};width:8px;border:none;}}
            QScrollBar::handle:vertical{{background:{C['border']};border-radius:4px;min-height:20px;}}
            QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{{height:0;}}
            QScrollBar:horizontal{{background:{C['panel']};height:8px;border:none;}}
            QScrollBar::handle:horizontal{{background:{C['border']};border-radius:4px;min-width:20px;}}
            QScrollBar::add-line:horizontal,QScrollBar::sub-line:horizontal{{width:0;}}""")

        root.addWidget(self._tree, stretch=1)

        # ── STATUS BAR ─────────────────────────────────────────────────────
        self._sb = QStatusBar()
        self._sb.setStyleSheet(f"""
            QStatusBar{{background:{C['header']};color:{C['text_dim']};
                font-family:Segoe UI;font-size:8pt;
                border-top:1px solid {C['border']};}}""")
        self._sb.showMessage("Ready"); self.setStatusBar(self._sb)

    # ───────────────────────────────────────────────────────────────────────
    # TIMER TICK  – refresh port list
    # ───────────────────────────────────────────────────────────────────────
    def _tick(self):
        self._refresh_ports()

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._update_port_list(ports)

    def _update_port_list(self, ports: list):
        cur = self._port_combo.currentText()
        self._port_combo.blockSignals(True)
        self._port_combo.clear()
        self._port_combo.addItems(ports)
        if cur in ports: self._port_combo.setCurrentText(cur)
        elif ports:      self._port_combo.setCurrentText(ports[0])
        self._port_combo.blockSignals(False)
    # ───────────────────────────────────────────────────────────────────────
    # CONNECTION
    # ───────────────────────────────────────────────────────────────────────
    def _toggle_connection(self):
        if self._connected: self._disconnect()
        else:               self._connect()

    def _connect(self):
        port = self._port_combo.currentText().strip()
        if not port:
            self._set_status("No port selected!", error=True); return
        try:
            baud = int(self._baud_combo.currentText().strip())
        except ValueError:
            self._set_status("Invalid baud rate", error=True); return
        try:
            self._ser = serial.Serial(port, baud, timeout=1, write_timeout=1,
                                      dsrdtr=False, rtscts=False)
            time.sleep(0.15)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

            # ── Real liveness check ──────────────────────────────────────
            # Send a DSC Default Session request and see if anything comes back
            # within 300 ms.  If nothing, still allow connection (ECU may be
            # sleeping) but warn the user.
            alive = self._check_alive()

            self._connected  = True
            self._start_time = time.monotonic()

            # Start the persistent reader thread
            self._reader_stop.clear()
            threading.Thread(target=self._reader_thread, daemon=True).start()

            # ── UI feedback ──────────────────────────────────────────────
            self._conn_btn.setStyleSheet(f"""
                QPushButton{{background:{C['accent_red']};color:#FFF;
                    border:none;border-radius:4px;padding:5px 14px;}}
                QPushButton:hover{{background:#A02020;}}""")
            self._conn_btn.setText("Disconnect")

            status_color = C["accent_green"] if alive else "#E67E00"
            status_text  = (f"● Connected  {port} @ {baud}"
                            if alive else
                            f"● Connected  {port} @ {baud}  (no ECU response)")
            self._conn_status.setText(status_text)
            self._conn_status.setStyleSheet(
                f"color:{status_color};background:transparent;")

            self._dot.setStyleSheet(
                f"color:{status_color};background:transparent;font-size:13px;")
            self._conn_lbl.setText("Connected" if alive else "Connected – ECU silent")
            self._conn_lbl.setStyleSheet(
                f"color:{status_color};background:transparent;")

            self._set_status(
                f"Port {port} opened at {baud} baud" +
                ("" if alive else "  ⚠ ECU did not respond to ping"))

        except serial.SerialException as e:
            if self._ser:
                try: self._ser.close()
                except: pass
                self._ser = None
            self._set_status(f"Cannot open {port}: {e}", error=True)

    def _check_alive(self) -> bool:
        """Send a DSC-Default ping and wait up to 400 ms for any reply."""
        try:
            ping = build_frame([0x10, 0x01])
            self._ser.write(ping)
            deadline = time.monotonic() + 0.4
            buf = b""
            while time.monotonic() < deadline:
                chunk = self._ser.read(FRAME_SIZE - len(buf))
                if chunk: buf += chunk
                if len(buf) >= FRAME_SIZE: return True
            return False
        except Exception:
            return False

    def _disconnect(self):
        self._reader_stop.set()
        time.sleep(0.05)
        if self._ser:
            try: self._ser.close()
            except: pass
            self._ser = None
        self._connected = False

        self._conn_btn.setStyleSheet(f"""
            QPushButton{{background:{C['btn_send']};color:{C['btn_send_fg']};
                border:none;border-radius:4px;padding:5px 14px;}}
            QPushButton:hover{{background:#00A870;}}""")
        self._conn_btn.setText("Connect")
        self._conn_status.setText("● Disconnected")
        self._conn_status.setStyleSheet(
            f"color:{C['accent_red']};background:transparent;")
        self._dot.setStyleSheet(
            "color:#AAAAAA;background:transparent;font-size:13px;")
        self._conn_lbl.setText("Not Connected")
        self._conn_lbl.setStyleSheet(
            f"color:{C['text_dim']};background:transparent;")
        self._set_status("Disconnected")

    # ───────────────────────────────────────────────────────────────────────
    # SEND
    # ───────────────────────────────────────────────────────────────────────
    def _send_request(self):
        raw = self._input.text().strip()
        if not raw: return
        if not self._connected or not self._ser:
            self._set_status("Not connected!", error=True); return

        payload = parse_input(raw)
        if not payload:
            self._set_status("Invalid hex input", error=True); return

        tx = build_frame(payload)
        t  = round(time.monotonic() - self._start_time, 3)
        self._add_row(t, *describe_frame(tx, "Client"))

        def _write():
            try:
                self._ser.write(tx)
            except Exception as e:
                self._status_sig.emit(f"Send error: {e}", True)

        threading.Thread(target=_write, daemon=True).start()
        self._input.clear()
        self._set_status(f"Sent → {' '.join(f'{b:02X}' for b in tx)}")

    # ───────────────────────────────────────────────────────────────────────
    # PERSISTENT BACKGROUND READER THREAD
    # ───────────────────────────────────────────────────────────────────────
    def _reader_thread(self):
        """
        Continuously reads FRAME_SIZE-byte frames from the serial port
        and emits _row_sig for every ECU frame received.
        Handles partial reads, port disconnection, and flush errors.
        """
        buf = b""
        while not self._reader_stop.is_set():
            try:
                if not self._ser or not self._ser.is_open:
                    break
                waiting = self._ser.in_waiting
                if waiting == 0:
                    time.sleep(0.005)
                    continue
                chunk = self._ser.read(min(waiting, FRAME_SIZE - len(buf)))
                if chunk:
                    buf += chunk
                while len(buf) >= FRAME_SIZE:
                    frame = buf[:FRAME_SIZE]
                    buf   = buf[FRAME_SIZE:]
                    t     = round(time.monotonic() - self._start_time, 3)
                    self._row_sig.emit(t, *describe_frame(frame, "ECU"))
            except serial.SerialException:
                self._status_sig.emit("⚠ Connection lost", True)
                break
            except Exception as e:
                self._status_sig.emit(f"Reader error: {e}", True)
                break
        # Auto-update UI if we fell out due to port loss
        if self._connected:
            self._status_sig.emit("Port disconnected unexpectedly", True)
            self._connected = False

    # ───────────────────────────────────────────────────────────────────────
    # ADD ROW TO TREE
    # ───────────────────────────────────────────────────────────────────────
    def _add_row(self, t: float, name: str, svc: str,
                 can: str, colored: list, sender: str):
        is_client = sender == "Client"
        is_nrc    = name   == "NegativeResponse"
        row_bg    = QColor(C["row_even"] if self._row_count%2==0 else C["row_odd"])
        self._row_count += 1

        item = QTreeWidgetItem(self._tree)
        for c in range(7): item.setBackground(c, QBrush(row_bg))
        item.setTextAlignment(COL_BYTES, Qt.AlignVCenter)

        ui_bold = QFont("Segoe UI", 8)
        ui_bold.setWeight(QFont.Normal)

        # ── Time ────────────────────────────────────────────────────────
        item.setText(COL_TIME, f"{t % 10.0:.3f}")
        item.setForeground(COL_TIME, QBrush(QColor(C["text"])))
        item.setFont(COL_TIME, ui_bold)
        item.setTextAlignment(COL_TIME, Qt.AlignRight|Qt.AlignVCenter)

        # ── Protocol Service ────────────────────────────────────────────
        proto_color = C["pci"] if is_nrc else C["text"]
        item.setText(COL_PROTO, name)
        item.setForeground(COL_PROTO, QBrush(QColor(proto_color)))
        item.setFont(COL_PROTO, ui_bold)
        # ── Service detail ──────────────────────────────────────────────
        if is_nrc:
            sc = C["pci"]   # red for error
        else:
            sc = C["text"]       # black  
        item.setText(COL_SVC, svc)
        item.setForeground(COL_SVC, QBrush(QColor(sc)))
        item.setFont(COL_SVC, ui_bold)
        # ── CAN ID ──────────────────────────────────────────────────────
        item.setText(COL_CAN, can)
        item.setForeground(COL_CAN,
            QBrush(QColor(C["can_client"] if is_client else C["can_ecu"])))
        item.setFont(COL_CAN, ui_bold)
        item.setTextAlignment(COL_CAN, Qt.AlignCenter|Qt.AlignVCenter)

        # ── Colored bytes (HTML delegate) ────────────────────────────────
        html  = bytes_html(colored)
        plain = " ".join(h for h,_ in colored)
        item.setData(COL_BYTES, Qt.UserRole, html)
        item.setData(COL_BYTES, Qt.BackgroundRole, QBrush(row_bg))
        item.setText(COL_BYTES, plain)     # plain text for copy/search
        item.setFont(COL_BYTES, ui_bold)

        # ── Sender ──────────────────────────────────────────────────────
        item.setText(COL_SENDER, "DiagBox" if is_client else "ECU")
        item.setForeground(COL_SENDER, QBrush(QColor(C["text"])))
        item.setFont(COL_SENDER, ui_bold)
        item.setTextAlignment(COL_SENDER, Qt.AlignCenter|Qt.AlignVCenter)

        # ── Frame type ───────────────────────────────────────────────────
        item.setText(COL_FRAME, "Single Frame (SF)")
        item.setForeground(COL_FRAME, QBrush(QColor(C["text"])))
        item.setFont(COL_FRAME, ui_bold)
        item.setTextAlignment(COL_FRAME, Qt.AlignCenter|Qt.AlignVCenter)

        self._tree.scrollToItem(item)

    # ───────────────────────────────────────────────────────────────────────
    # CLEAR / STATUS
    # ───────────────────────────────────────────────────────────────────────
    def _clear_trace(self):
        self._tree.clear(); self._row_count = 0
        self._set_status("Trace cleared")

    def _set_status(self, msg: str, error: bool = False):
        col = C["accent_red"] if error else C["text_dim"]
        self._sb.setStyleSheet(f"""
            QStatusBar{{background:{C['header']};color:{col};
                font-family:Segoe UI;font-size:8pt;
                border-top:1px solid {C['border']};}}""")
        self._sb.showMessage(msg)

    def closeEvent(self, event):
        self._reader_stop.set()
        if self._ser:
            try: self._ser.close()
            except: pass
        event.accept()


# ═══════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(C["bg"]))
    pal.setColor(QPalette.WindowText,      QColor(C["text"]))
    pal.setColor(QPalette.Base,            QColor(C["input_bg"]))
    pal.setColor(QPalette.AlternateBase,   QColor(C["row_odd"]))
    pal.setColor(QPalette.Text,            QColor(C["text"]))
    pal.setColor(QPalette.Button,          QColor(C["panel"]))
    pal.setColor(QPalette.ButtonText,      QColor(C["text"]))
    pal.setColor(QPalette.Highlight,       QColor("#BBDEFB"))
    pal.setColor(QPalette.HighlightedText, QColor(C["text"]))
    app.setPalette(pal)

    win = SigmaUDSApp()
    win.show()
    sys.exit(app.exec_())