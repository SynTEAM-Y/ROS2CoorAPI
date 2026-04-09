#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ProjectN • Queue Types (Q/R/D vocabulary, T1/T2/T3 ready)

──────────────────────────────────────────────────────────────────────────────
INTRODUCTION (plain-English overview)
──────────────────────────────────────────────────────────────────────────────
This module defines tiny, typed containers that the pipeline uses internally.
They are *just data shapes* (no ROS calls, no networking) so the three threads
(T1/T2/T3) can talk about work in a consistent, type-safe way.

Two things we model here:

1) CONTROL work → CtrlWork
   • "Do something" tasks triggered by services, e.g.:
       - GET_MSG_ID (Q): agent asks the root for a new id
       - SUBMIT_DM  (D): agent submits a Data message using a granted id
   • These flow through the control lane and are executed by T3 when they reach
     the head of the input FIFO.

2) DATA items → DMItem
   • Wrappers around *data messages* that must be delivered in strict id order.
   • The pipeline puts these into a PriorityQueue using the message id as the
     priority, so T3 can deliver them when id == expected_id.

What’s NOT here:
- No ROS 2 logic, no services, no publishers/subscribers.
- Only small dataclasses and enums with timestamps, priorities, etc.

How it fits with the pipeline:
- T1 merges control work and incoming messages into one FIFO (processor.py).
- T2 pops Data from the FIFO and moves it into a PriorityQueue by id.
- T3 delivers Data in order; otherwise it processes Control at the FIFO head.

Back-compat:
- We export aliases WorkItem (CtrlWork) and DataItem (DMItem) so older code
  that uses the legacy names still runs.

No side-effects:
- Importing this module does not start threads or change global state.
"""

from __future__ import annotations  # allow forward references in type hints

# dataclass: easy "plain data object" with fields and defaults
from dataclasses import dataclass, field
# Enum: named constants (stable values + clearer logs)
from enum import Enum
# typing: hints for readability and editors (no runtime behavior)
from typing import Any
# time: monotonic timestamp for tracing and (rare) tie-breakers
import time

# ==============================================================================
# TYPES AND CONSTANTS (no side effects)
# ==============================================================================

class CtrlKind(str, Enum):
    """
    Kinds of CONTROL work the pipeline understands.

    Why a *string* Enum?
    - The value is self-explanatory in logs/JSON ("GET_MSG_ID" vs 1).
    - Easier to scan telemetry and debug prints.

    Members:
      GET_MSG_ID : "Q" operation — request a unique message id from the root.
      SUBMIT_DM  : "D-up"       — submit a data message using a granted id.

    Aliases (for older code):
      Q_REQUEST_ID ≡ GET_MSG_ID
      SUBMIT_DATA  ≡ SUBMIT_DM
    """
    GET_MSG_ID = "GET_MSG_ID"
    SUBMIT_DM = "SUBMIT_DM"

    # Backward-compatible aliases (keep old callers working)
    Q_REQUEST_ID = "GET_MSG_ID"
    SUBMIT_DATA = "SUBMIT_DM"


@dataclass(frozen=True)
class CtrlWork:
    """
    CONTROL WORK ITEM (immutable)

    What it represents:
      A single unit of "do something" work for the node (handled by T3 through
      node._pipeline_handle_ctrl). Example payloads:
        • (req, fut) for GET_MSG_ID
        • (req, fut) for SUBMIT_DM

    Why frozen=True?
      - Safety: once enqueued, it should never be mutated by other threads.

    Fields:
      kind       : CtrlKind — what action to perform.
      payload    : Any      — data needed to perform it; often a tuple (req, fut).
      created_ts : float    — when it was created (monotonic seconds) for tracing.
    """
    kind: CtrlKind
    payload: Any
    created_ts: float = field(default_factory=lambda: time.monotonic())


@dataclass(order=True, frozen=True)
class DMItem:
    """
    DATA ITEM (immutable, orderable)

    Purpose:
      Wrap a data message so the pipeline can put it in a PriorityQueue and
      have it sort by message id (strict in-order delivery).

    Why order=True?
      - Instances are comparable using field order; here we set the sort keys
        to (prio, created_ts) via field order and compare=False on the payload.

    Ordering keys (in this declaration order):
      1) prio       : int   → message id; lower id delivers first
      2) created_ts : float → tie-breaker if ids are the same (rare)

    Fields:
      prio       : int   — the message id used as priority.
      created_ts : float — wrap time (monotonic).
      payload    : Any   — the original message object; excluded from comparisons.
    """
    prio: int
    created_ts: float
    payload: Any = field(compare=False)


# ==============================================================================
# SMALL CONVENIENCE CONSTRUCTORS (no logic; just nicer call sites)
# ==============================================================================

def make_ctrl(kind: CtrlKind, payload: Any) -> CtrlWork:
    """
    Build a CtrlWork with an auto-filled timestamp.

    Rationale:
      You could call CtrlWork(...) directly, but this helper keeps call sites
      short and consistent across the codebase.
    """
    return CtrlWork(kind=kind, payload=payload)

def make_dm(msg_id: int, payload: Any) -> DMItem:
    """
    Build a DMItem whose priority equals the message id.

    Assumptions:
      - msg_id is a positive integer (validated by protocol edges upstream).
      - This function deliberately does not re-validate for speed/simplicity.
    """
    return DMItem(prio=int(msg_id), created_ts=time.monotonic(), payload=payload)


# ==============================================================================
# BACKWARD-COMPATIBLE EXPORTS (do not remove)
# ==============================================================================

# Legacy names kept for older imports
WorkItem = CtrlWork   # alias: WorkItem → CtrlWork
DataItem = DMItem     # alias: DataItem → DMItem

# Public API surface (for "from queues import *")
__all__ = [
    # Preferred names
    "CtrlKind", "CtrlWork", "DMItem",
    # Helpers
    "make_ctrl", "make_dm",
    # Back-compat aliases
    "WorkItem", "DataItem",
]
