#!/usr/bin/env python3
from PyQt5 import QtWidgets, QtCore
from openpilot.common.params import Params

class TurnSlowdownPanel(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.params = Params()

        layout = QtWidgets.QVBoxLayout()

        # Toggle
        self.toggle = QtWidgets.QCheckBox("Enable Slowdown in Turns")
        self.toggle.setChecked(self.params.get_bool("TinklaTurnSlowdownEnabled"))
        self.toggle.stateChanged.connect(self.on_toggle)
        layout.addWidget(self.toggle)

        # Factor slider
        self.factor_label = QtWidgets.QLabel()
        self.factor_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.factor_slider.setMinimum(50)   # 0.5
        self.factor_slider.setMaximum(200)  # 2.0
        self.factor_slider.setValue(int(self.params.get_float("TinklaTurnSlowdownFactor", 1.0) * 100))
        self.factor_slider.valueChanged.connect(self.on_factor_changed)
        layout.addWidget(self.factor_label)
        layout.addWidget(self.factor_slider)

        self.setLayout(layout)
        self.update_factor_label()

    def on_toggle(self, state):
        enabled = state == QtCore.Qt.Checked
        self.params.put_bool("TinklaTurnSlowdownEnabled", enabled)

    def on_factor_changed(self, value):
        factor = value / 100.0
        self.params.put_float("TinklaTurnSlowdownFactor", factor)
        self.update_factor_label()

    def update_factor_label(self):
        factor = self.factor_slider.value() / 100.0
        self.factor_label.setText(f"Slowdown Factor: {factor:.2f}")
