#!/usr/bin/env python3
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

from PyQt5 import QtWidgets, QtCore
from Rosmaster_Lib import Rosmaster

EFFECTS = {
    0: "stop",
    1: "running_water",
    2: "marquee",
    3: "breathing",
    4: "gradient",
    5: "starlight",
    6: "battery",
}

class RosmasterRgbNode(Node):
    def __init__(self):
        super().__init__('rosmaster_rgb_gui')
        self.bot = Rosmaster(debug=False)
        self.bot.create_receive_threading()
        self.bot.set_car_type(self.bot.CARTYPE_X3_PLUS)

        self.declare_parameter('effect', 1)
        self.declare_parameter('speed', 5)   # 1..10 smaller is faster
        self.declare_parameter('parm', 255)  # 0..255

        self.apply_effect_from_params()

        self.add_on_set_parameters_callback(self._on_params_changed)

        # Topic for external LED commands if you still want it
        self.sub_led = self.create_subscription(
            Int32MultiArray, 'set_led', self.on_set_led, 10
        )
        # Service to stop effects
        self.srv_stop = self.create_service(Trigger, 'stop_effects', self.on_stop)

        self.get_logger().info('Rosmaster RGB GUI node ready')

    # ----- params -----
    def _on_params_changed(self, params):
        for p in params:
            if p.name == 'effect' and not (0 <= int(p.value) <= 6):
                return SetParametersResult(successful=False, reason='effect must be 0..6')
            if p.name == 'speed' and not (1 <= int(p.value) <= 10):
                return SetParametersResult(successful=False, reason='speed must be 1..10')
            if p.name == 'parm' and not (0 <= int(p.value) <= 255):
                return SetParametersResult(successful=False, reason='parm must be 0..255')
        ok = self.apply_effect_from_params()
        return SetParametersResult(successful=ok, reason='' if ok else 'apply failed')

    def apply_effect_from_params(self):
        eff = self.get_parameter('effect').get_parameter_value().integer_value
        spd = self.get_parameter('speed').get_parameter_value().integer_value
        par = self.get_parameter('parm').get_parameter_value().integer_value
        return self.set_effect(eff, spd, par)

    # ----- device ops -----
    def set_effect(self, effect: int, speed: int, parm: int):
        try:
            self.bot.set_colorful_effect(int(effect), speed=int(speed), parm=int(parm))
            self.get_logger().info(f'Effect {effect} {EFFECTS.get(effect,"")}, speed={speed}, parm={parm}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to set effect: {e}')
            return False

    def set_led(self, led_id: int, r: int, g: int, b: int):
        try:
            # Stop effects for direct LED control
            self.bot.set_colorful_effect(0, speed=1, parm=0)
            self.bot.set_colorful_lamps(int(led_id), int(r), int(g), int(b))
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to set LED: {e}')
            return False

    def stop_effects(self):
        try:
            self.bot.set_colorful_effect(0, speed=1, parm=0)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to stop effects: {e}')
            return False

    # ----- ROS interfaces kept for compatibility -----
    def on_set_led(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) != 4:  # [led_id, r, g, b]
            self.get_logger().error('set_led expects [led_id, r, g, b]')
            return
        self.set_led(*data)

    def on_stop(self, _, response):
        ok = self.stop_effects()
        response.success = ok
        response.message = 'Effects stopped' if ok else 'Failed to stop'
        return response


# ---------------- GUI ----------------
class RgbControlWindow(QtWidgets.QWidget):
    def __init__(self, node: RosmasterRgbNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('Rosmaster RGB Control')

        # Effect selector
        self.effectBox = QtWidgets.QComboBox()
        for k in sorted(EFFECTS.keys()):
            self.effectBox.addItem(f'{k} - {EFFECTS[k]}', k)

        # Speed 1..10
        self.speedSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speedSlider.setRange(1, 10)
        self.speedVal = QtWidgets.QSpinBox()
        self.speedVal.setRange(1, 10)

        # Parm 0..255
        self.parmSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.parmSlider.setRange(0, 255)
        self.parmVal = QtWidgets.QSpinBox()
        self.parmVal.setRange(0, 255)

        # LED id
        self.ledId = QtWidgets.QSpinBox()
        self.ledId.setRange(0, 255)
        self.ledId.setValue(255)  # 255 means all LEDs

        # RGB sliders 0..255
        self.rSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.rSlider.setRange(0, 255)
        self.gSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.gSlider.setRange(0, 255)
        self.bSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.bSlider.setRange(0, 255)
        self.rVal = QtWidgets.QSpinBox(); self.rVal.setRange(0, 255)
        self.gVal = QtWidgets.QSpinBox(); self.gVal.setRange(0, 255)
        self.bVal = QtWidgets.QSpinBox(); self.bVal.setRange(0, 255)

        # Buttons
        self.applyEffectBtn = QtWidgets.QPushButton('Apply Effect')
        self.stopBtn = QtWidgets.QPushButton('Stop Effects')
        self.setLedBtn = QtWidgets.QPushButton('Set LED Color')
        self.allOffBtn = QtWidgets.QPushButton('All Off')

        # Layout
        grid = QtWidgets.QGridLayout(self)
        row = 0
        grid.addWidget(QtWidgets.QLabel('Effect'), row, 0); grid.addWidget(self.effectBox, row, 1, 1, 3); row += 1
        grid.addWidget(QtWidgets.QLabel('Speed'), row, 0); grid.addWidget(self.speedSlider, row, 1, 1, 2); grid.addWidget(self.speedVal, row, 3); row += 1
        grid.addWidget(QtWidgets.QLabel('Parm'), row, 0); grid.addWidget(self.parmSlider, row, 1, 1, 2); grid.addWidget(self.parmVal, row, 3); row += 1
        grid.addWidget(self.applyEffectBtn, row, 1); grid.addWidget(self.stopBtn, row, 2); row += 1
        # grid.addWidget(QtWidgets.QLabel('LED ID (0..16 or 255=all)'), row, 0); grid.addWidget(self.ledId, row, 1); row += 1
        grid.addWidget(QtWidgets.QLabel('R'), row, 0); grid.addWidget(self.rSlider, row, 1, 1, 2); grid.addWidget(self.rVal, row, 3); row += 1
        grid.addWidget(QtWidgets.QLabel('G'), row, 0); grid.addWidget(self.gSlider, row, 1, 1, 2); grid.addWidget(self.gVal, row, 3); row += 1
        grid.addWidget(QtWidgets.QLabel('B'), row, 0); grid.addWidget(self.bSlider, row, 1, 1, 2); grid.addWidget(self.bVal, row, 3); row += 1
        grid.addWidget(self.setLedBtn, row, 1); grid.addWidget(self.allOffBtn, row, 2)

        # Wire up pairs
        self.speedSlider.valueChanged.connect(self.speedVal.setValue)
        self.speedVal.valueChanged.connect(self.speedSlider.setValue)
        self.parmSlider.valueChanged.connect(self.parmVal.setValue)
        self.parmVal.valueChanged.connect(self.parmSlider.setValue)
        self.rSlider.valueChanged.connect(self.rVal.setValue); self.rVal.valueChanged.connect(self.rSlider.setValue)
        self.gSlider.valueChanged.connect(self.gVal.setValue); self.gVal.valueChanged.connect(self.gSlider.setValue)
        self.bSlider.valueChanged.connect(self.bVal.setValue); self.bVal.valueChanged.connect(self.bSlider.setValue)

        # Buttons
        self.applyEffectBtn.clicked.connect(self.on_apply_effect)
        self.stopBtn.clicked.connect(self.on_stop)
        self.setLedBtn.clicked.connect(self.on_set_led)
        self.allOffBtn.clicked.connect(self.on_all_off)

        # Initialize from node params
        eff = node.get_parameter('effect').get_parameter_value().integer_value
        spd = node.get_parameter('speed').get_parameter_value().integer_value
        par = node.get_parameter('parm').get_parameter_value().integer_value
        idx = max(0, list(EFFECTS.keys()).index(eff)) if eff in EFFECTS else 0
        self.effectBox.setCurrentIndex(idx)
        self.speedSlider.setValue(spd)
        self.parmSlider.setValue(par)
        self.rSlider.setValue(0); self.gSlider.setValue(0); self.bSlider.setValue(0)

    # Slots
    def on_apply_effect(self):
        effect = int(self.effectBox.currentData())
        speed  = int(self.speedSlider.value())
        parm   = int(self.parmSlider.value())

        # Update ROS params so external tools see the change
        self.node.set_parameters([
            Parameter('effect', Parameter.Type.INTEGER, effect),
            Parameter('speed',  Parameter.Type.INTEGER, speed),
            Parameter('parm',   Parameter.Type.INTEGER, parm),
        ])

        # Apply to hardware
        self.node.set_effect(effect, speed, parm)


    def on_stop(self):
        self.node.stop_effects()

    def on_set_led(self):
        led_id = self.ledId.value()
        r, g, b = self.rSlider.value(), self.gSlider.value(), self.bSlider.value()
        self.node.set_led(led_id, r, g, b)

    def on_all_off(self):
        self.ledId.setValue(255)
        self.rSlider.setValue(0); self.gSlider.setValue(0); self.bSlider.setValue(0)
        self.node.set_led(255, 0, 0, 0)


def _spin_node_in_thread(node: Node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = RosmasterRgbNode()

    # Spin ROS in a background thread so parameters and services work
    spin_thread = threading.Thread(target=_spin_node_in_thread, args=(node,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = RgbControlWindow(node)
    win.resize(520, 320)
    win.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
