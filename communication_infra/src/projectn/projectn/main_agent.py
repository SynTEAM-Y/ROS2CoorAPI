#!/usr/bin/env python3
# One-shot agent: get ID (root grants), then submit DM; exit.
import sys
import json
import rclpy
from rclpy.node import Node
from projectn_interfaces.srv import GetMsgId, SubmitDm
from .utils import elog


class FakeAgent(Node):
    """
    Usage:
      ros2 run projectn agent <AgentName> <AttachNode> [optional message text...]

    Example:
      ros2 run projectn agent Agent2 A1 "hello from Agent2"
    """
    def __init__(self, agent='bot1', attach_node='A3', timeout_ms=15000):
        super().__init__('fake_agent')
        self.agent = agent
        self.attach_node = attach_node
        self.timeout_ms = int(timeout_ms)

        # Service clients on the attach node
        self.getid = self.create_client(GetMsgId, f'/{attach_node}/get_msg_id')
        self.submit = self.create_client(SubmitDm, f'/{attach_node}/submit_dm')

        self.get_logger().info(
            elog('AGENT', f"{agent}: waiting for /{attach_node}/get_msg_id & /{attach_node}/submit_dm ...")
        )

        # Be generous with discovery waits (matches the longer budgets)
        ok1 = self.getid.wait_for_service(timeout_sec=15.0)
        ok2 = self.submit.wait_for_service(timeout_sec=15.0)
        if not (ok1 and ok2):
            self.get_logger().error(elog('ERR',
                f"{self.agent}: services not available on {self.attach_node}"))
            raise RuntimeError("services unavailable")

    def run_once(self, text_payload: str = ""):
        # -------- Request ID (root gates) --------
        req = GetMsgId.Request()
        req.agent_name = self.agent
        req.path_json = "[]"  # start the upstream path
        req.timeout_ms_remaining = self.timeout_ms  # <-- longer budget baked in

        fut = self.getid.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=self.timeout_ms / 1000.0)
        if fut.result() is None:
            self.get_logger().error(elog('ERR', f"{self.agent}: get_id timeout"))
            return
        resp = fut.result()
        if not resp.ok:
            self.get_logger().error(elog('ERR', f"{self.agent}: get_id failed: {resp.reason}"))
            return

        msg_id = int(resp.msg_id)

        # Pretty-print upstream path (how we reached the root for GetMsgId)
        try:
            up_path = json.loads(resp.path_json_return)
        except Exception:
            up_path = []
        self.get_logger().info(
            elog('AGENT', f"{self.agent}: granted id={msg_id} via GetMsgId up_path={up_path}")
        )

        # -------- Submit DM (root will broadcast) --------
        sreq = SubmitDm.Request()
        sreq.msg_id = msg_id
        sreq.src_agent = self.agent
        sreq.payload_json = (
            text_payload if text_payload
            else json.dumps({"from": self.agent, "text": f"hello {msg_id}"})
        )

        sfut = self.submit.call_async(sreq)
        rclpy.spin_until_future_complete(self, sfut, timeout_sec=self.timeout_ms / 1000.0)

        if sfut.result() is None:
            self.get_logger().error(elog('ERR', f"{self.agent}: submit_dm timeout"))
            return

        sresp = sfut.result()
        if not sresp.ok:
            self.get_logger().error(elog('ERR', f"{self.agent}: submit_dm failed: {sresp.reason}"))
        else:
            self.get_logger().info(
                elog('AGENT', f"{self.agent}: submit_dm OK (root will broadcast with paths in DataMsg.path_json)")
            )


def main():
    rclpy.init()
    # Args: AgentName, AttachNode, (optional) message text...
    agent_name = sys.argv[1] if len(sys.argv) > 1 else 'bot1'
    attach_node = sys.argv[2] if len(sys.argv) > 2 else 'A3'
    # allow an optional free-form message after the two required args
    text_payload = " ".join(sys.argv[3:]) if len(sys.argv) > 3 else ""

    try:
        agent = FakeAgent(agent=agent_name, attach_node=attach_node, timeout_ms=15000)
        agent.run_once(text_payload=text_payload)
    finally:
        try:
            agent.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
