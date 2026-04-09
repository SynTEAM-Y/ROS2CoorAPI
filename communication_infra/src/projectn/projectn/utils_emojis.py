# -*- coding: utf-8 -*-
"""
Tiny emoji logger utilities for ProjectN.

Use:
  from ..utils_emojis import elog, TAG
  self.get_logger().info(elog(TAG.OK, f"{self.name}: ..."))
"""

from datetime import datetime

class TAG:
    # pipeline (processor.py)
    T1   = "📥"   # ingress
    T2   = "🧪"   # classification / staging
    T3   = "🚦"   # global order gate / deliver
    OK   = "✅"
    WARN = "⚠️"
    DROP = "🗑️"
    HOLD = "⏸️"

    # control / node (tree_node.py)
    GRANT = "🎟️"
    UP    = "⬆️"
    DOWN  = "⬇️"
    CAST  = "📣"
    INBOX = "📬"
    
# Flows
    Q_UP   = "🟦"   # Q forwarded upward (service hop)
    R_IN   = "🟨"   # R arriving (reply down)
    R_OUT  = "🟨"   # R leaving (next hop down)
    D_IN   = "📥"   # D delivered locally at node (before forwarding)
    D_UP   = "⬆️"   # D forwarded up to parent (service)
    D_DOWN = "⬇️"   # D broadcast down to child(ren) (topic)
    # hub / REPL (agent_hub.py)
    HUB   = "🧭"
    AG    = "🤖"
    SEND  = "📤"
    RECV  = "📥"
    TABLE = "📊"
    BAD   = "🛑"

def _ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def elog(tag: str, msg: str) -> str:
    """Compose a one-line log with timestamp + emoji tag."""
    return f"{_ts()} {tag} {msg}"
