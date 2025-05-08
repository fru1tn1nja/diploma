#!/usr/bin/env python3
"""ROS 2 Humble node: switch_logic_node
---------------------------------------
Модуль реализует конечный автомат плавного переключения управления
между ИИ‑модулем и оператором.

Состояния:
    0 – AI_CONTROL      (полный автопилот)
    1 – OPERATOR_CONTROL (полный ручной)
    2 – MIXED            (переход / совместное)
    3 – FAILSAFE         (аварийный режим)

Топики
------
Подписка:
  * /ai/events            : std_msgs/String  – JSON {"type": str, "severity": int, ...}
  * /operator/commands    : std_msgs/String  – plain: TAKE_OVER | RESUME_AI | FAILSAFE
  * /telemetry/health     : std_msgs/UInt8   – 0/1 (0 = lost)

Публикация:
  * /control/mode         : std_msgs/UInt8   – код состояния (см. Enum)

Параметры (declare_parameter):
  switch_timeout_sec: float = 5.0   # потеря связи
  ai_confidence_min : float = 0.6   # порог доверия

Copyright © 2025
MIT License (см. репозиторий)
"""

from __future__ import annotations

import json
from enum import Enum, IntEnum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String, UInt8, Float32


class ControlMode(IntEnum):
    AI_CONTROL = 0
    OPERATOR_CONTROL = 1
    MIXED = 2
    FAILSAFE = 3


class SwitchLogic(Node):
    def __init__(self) -> None:
        super().__init__("switch_logic")

        # Parameters
        self.switch_timeout = self.declare_and_get_parameter("switch_timeout_sec", 5.0)
        self.ai_confidence_min = self.declare_and_get_parameter("ai_confidence_min", 0.6)

        # Internal state
        self._mode: ControlMode = ControlMode.AI_CONTROL
        self._last_operator_command: Optional[str] = None
        self._last_ai_event: Optional[str] = None
        self._last_health_ts: Time = self.get_clock().now()

        # Pub/Sub
        self.create_subscription(String, "/ai/events", self.on_ai_event, 10)
        self.create_subscription(String, "/operator/commands", self.on_operator_command, 10)
        self.create_subscription(UInt8, "/telemetry/health", self.on_health, 10)
        self._pub_mode = self.create_publisher(UInt8, "/control/mode", 10)

        # Timer – periodic check
        self.create_timer(0.2, self.evaluate_state)

        self.get_logger().info("SwitchLogic node started → initial mode: AI_CONTROL")

    # ------------------------------------------------------------
    # Parameter helper
    def declare_and_get_parameter(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().double_value

    # ------------------------------------------------------------
    # Callbacks
    def on_ai_event(self, msg: String):
        self._last_ai_event = msg.data
        try:
            data = json.loads(msg.data)
            severity = data.get("severity", 0)
            conf = data.get("confidence", 1.0)
        except json.JSONDecodeError:
            self.get_logger().warning("Bad AI event JSON: %s", msg.data)
            return

        if conf < self.ai_confidence_min:
            self.transition(ControlMode.OPERATOR_CONTROL, reason="low AI confidence")
        elif severity >= 2:  # критическая ситуация
            self.transition(ControlMode.OPERATOR_CONTROL, reason="critical AI event")

    def on_operator_command(self, msg: String):
        self._last_operator_command = msg.data
        if msg.data == "TAKE_OVER":
            self.transition(ControlMode.OPERATOR_CONTROL, reason="operator TAKE_OVER")
        elif msg.data == "RESUME_AI":
            self.transition(ControlMode.AI_CONTROL, reason="operator RESUME_AI")
        elif msg.data == "FAILSAFE":
            self.transition(ControlMode.FAILSAFE, reason="operator FAILSAFE")

    def on_health(self, msg: UInt8):
        if msg.data:
            self._last_health_ts = self.get_clock().now()
        # потеря связи проверяется в evaluate_state

    # ------------------------------------------------------------
    # Main FSM evaluation
    def evaluate_state(self):
        now = self.get_clock().now()
        if (now - self._last_health_ts) > Duration(seconds=float(self.switch_timeout)):
            self.transition(ControlMode.FAILSAFE, reason="health timeout")
        # publish current mode on every cycle
        self._pub_mode.publish(UInt8(data=int(self._mode)))

    # ------------------------------------------------------------
    # Transition helper
    def transition(self, new_mode: ControlMode, *, reason: str):
        if new_mode == self._mode:
            return  # no change
        self.get_logger().warning(f"MODE {self._mode.name} → {new_mode.name} ({reason})")
        # TODO: добавить логику плавного перехода (буферизация/интерполяция команд)
        self._mode = new_mode


# -----------------------------------------------------------------
# Entry‑point

def main(args=None):
    rclpy.init(args=args)
    node = SwitchLogic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown by CTRL‑C")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
