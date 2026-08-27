#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ImageView — a QLabel that shows a BGR frame and lets the operator drag an ROI.

Pure Qt. No rospy: it is handed numpy arrays by the window and knows nothing
about where they came from.

The ROI is stored in IMAGE coordinates, not widget coordinates. The reference
UI stored the widget rectangle it had drawn, so the saved region moved whenever
the window was resized and meant something different on a 5472x3648 Basler
frame than on the 1280x720 preview it was drawn over. Every conversion goes
through _to_image / _to_widget here.
"""

import json
import os

import cv2
import numpy as np
from PyQt5.QtCore import QPoint, QRect, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import QLabel, QSizePolicy


class ImageView(QLabel):
    """Frame display with optional drag-to-select ROI."""

    roi_changed = pyqtSignal(object)  # QRect in image coords, or None
    double_clicked = pyqtSignal(str)  # this view's title

    def __init__(self, title='', roi_enabled=False, roi_config=None,
                 parent=None):
        super().__init__(parent)
        # Small enough to work as a thumbnail in the bottom strip; the main
        # views get their size from layout stretch, not from this.
        self.setMinimumSize(160, 120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet('background:#1b1b1b; color:#888; border:1px solid #444;')
        self.setText(title or 'no signal')

        self._title = title
        self._roi_enabled = roi_enabled
        self._roi_config = roi_config
        self._frame = None            # last BGR frame, full resolution
        self._scaled_rect = QRect()   # where the pixmap actually sits
        self._roi = None              # QRect in image coords
        self._drag_start = None
        self._drag_now = None
        self._centre_box = 0          # side length in image px, 0 = off

        if roi_config:
            self._load_roi()

    # ---------- frame ----------
    def set_frame(self, bgr):
        """Show a BGR numpy frame. Cheap enough to call at camera rate."""
        self._frame = bgr
        self.update()

    def frame(self):
        return self._frame

    def set_centre_box(self, side_px):
        """Draw a guide box of `side_px` IMAGE pixels centred on the frame.

        The reference UI drew a fixed 500 px box to aim the operator at the
        region inference would later crop. Expressed in image pixels so it
        keeps meaning the same thing as the widget is resized.
        """
        self._centre_box = int(side_px)
        self.update()

    def cropped_roi(self):
        """The current frame cropped to the ROI, or the whole frame if none."""
        if self._frame is None:
            return None
        if self._roi is None or self._roi.isNull():
            return self._frame
        h, w = self._frame.shape[:2]
        x0 = max(0, min(w - 1, self._roi.left()))
        y0 = max(0, min(h - 1, self._roi.top()))
        x1 = max(x0 + 1, min(w, self._roi.right() + 1))
        y1 = max(y0 + 1, min(h, self._roi.bottom() + 1))
        return self._frame[y0:y1, x0:x1]

    def roi(self):
        return self._roi

    # ---------- painting ----------
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.black)

        if self._frame is None:
            painter.setPen(QPen(Qt.gray))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             self._title or 'no signal')
            painter.end()
            return

        # A mono frame is drawn as Grayscale8 directly. Expanding it to RGB
        # just to hand Qt three identical channels would cost a 40 MB copy per
        # frame on the wrist camera, on the GUI thread, for no visible
        # difference.
        if self._frame.ndim == 2:
            buf = np.ascontiguousarray(self._frame)
            h, w = buf.shape
            qimg = QImage(buf.data, w, h, buf.strides[0],
                          QImage.Format_Grayscale8)
        else:
            buf = np.ascontiguousarray(
                cv2.cvtColor(self._frame, cv2.COLOR_BGR2RGB))
            h, w = buf.shape[:2]
            qimg = QImage(buf.data, w, h, buf.strides[0], QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)

        x = (self.width() - pix.width()) // 2
        y = (self.height() - pix.height()) // 2
        self._scaled_rect = QRect(x, y, pix.width(), pix.height())
        painter.drawPixmap(x, y, pix)

        if self._centre_box > 0:
            side = self._centre_box
            box = QRect((w - side) // 2, (h - side) // 2, side, side)
            painter.setPen(QPen(Qt.green, 2))
            painter.drawRect(self._to_widget(box))

        if self._drag_start is not None and self._drag_now is not None:
            painter.setPen(QPen(Qt.yellow, 2, Qt.DashLine))
            painter.drawRect(QRect(self._drag_start, self._drag_now).normalized())
        elif self._roi is not None and not self._roi.isNull():
            painter.setPen(QPen(Qt.cyan, 2))
            painter.drawRect(self._to_widget(self._roi))

        painter.setPen(QPen(Qt.white))
        painter.drawText(self.rect().adjusted(6, 4, -6, -4),
                         Qt.AlignTop | Qt.AlignLeft,
                         f'{self._title}  {w}x{h}')
        painter.end()

    # ---------- coordinate conversion ----------
    def _to_image(self, point):
        """Widget point -> image pixel. None when outside the drawn pixmap."""
        if self._frame is None or self._scaled_rect.width() <= 0:
            return None
        h, w = self._frame.shape[:2]
        rel_x = (point.x() - self._scaled_rect.x()) / self._scaled_rect.width()
        rel_y = (point.y() - self._scaled_rect.y()) / self._scaled_rect.height()
        rel_x = min(max(rel_x, 0.0), 1.0)
        rel_y = min(max(rel_y, 0.0), 1.0)
        return QPoint(int(rel_x * w), int(rel_y * h))

    def _to_widget(self, rect):
        """Image rect -> widget rect, for drawing."""
        if self._frame is None or self._scaled_rect.width() <= 0:
            return QRect()
        h, w = self._frame.shape[:2]
        sx = self._scaled_rect.width() / float(w)
        sy = self._scaled_rect.height() / float(h)
        return QRect(
            int(self._scaled_rect.x() + rect.x() * sx),
            int(self._scaled_rect.y() + rect.y() * sy),
            max(1, int(rect.width() * sx)),
            max(1, int(rect.height() * sy)),
        )

    # ---------- mouse ----------
    def mousePressEvent(self, event):
        if not self._roi_enabled:
            return
        if event.button() == Qt.LeftButton:
            self._drag_start = event.pos()
            self._drag_now = event.pos()
            self.update()
        elif event.button() == Qt.RightButton:
            self._roi = None
            self._save_roi()
            self.roi_changed.emit(None)
            self.update()

    def mouseDoubleClickEvent(self, event):
        """Ask the window to blow this view up to fill the panel.

        Letterboxing is the reason this exists. A 5472x3648 Basler frame is 3:2,
        and no fixed grid cell matches that, so a share of every pane is black
        bars — measured at 29% waste in the original near-square cell. Rather
        than tuning stretch factors for one camera's aspect and getting it wrong
        for the next, let the operator give one view the whole area when they
        are actually aiming with it.
        """
        # Left button only: the right button clears the ROI, and a double
        # right-click should not also change the layout.
        if event.button() == Qt.LeftButton:
            self.double_clicked.emit(self._title)

    def mouseMoveEvent(self, event):
        if self._drag_start is not None:
            self._drag_now = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if not self._roi_enabled or self._drag_start is None:
            return
        if event.button() != Qt.LeftButton:
            return
        start = self._to_image(self._drag_start)
        end = self._to_image(event.pos())
        self._drag_start = None
        self._drag_now = None
        if start is None or end is None:
            self.update()
            return
        rect = QRect(start, end).normalized()
        # A stray click is a 0x0 rect; treat anything degenerate as "clear"
        # rather than storing a region that crops to nothing.
        self._roi = rect if rect.width() > 4 and rect.height() > 4 else None
        self._save_roi()
        self.roi_changed.emit(self._roi)
        self.update()

    # ---------- persistence ----------
    def _load_roi(self):
        try:
            if not os.path.exists(self._roi_config):
                return
            with open(self._roi_config, 'r') as f:
                data = json.load(f)
            if all(k in data for k in ('x', 'y', 'w', 'h')):
                self._roi = QRect(data['x'], data['y'], data['w'], data['h'])
        except Exception:
            # A corrupt ROI file must not stop the window opening.
            self._roi = None

    def _save_roi(self):
        if not self._roi_config:
            return
        try:
            os.makedirs(os.path.dirname(self._roi_config), exist_ok=True)
            with open(self._roi_config, 'w') as f:
                if self._roi is None:
                    json.dump({}, f)
                else:
                    json.dump({'x': self._roi.x(), 'y': self._roi.y(),
                               'w': self._roi.width(), 'h': self._roi.height()},
                              f, indent=2)
        except Exception:
            pass
