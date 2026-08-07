#!/usr/bin/env python3
"""Block until a TF is available (and, under sim time, until /clock is flowing).

Why this exists
---------------
Nav2's costmaps refuse to activate without ``<global_frame> -> <robot_frame>``,
and a failure there is not recoverable: lifecycle_manager logs

    Failed to activate local_costmap because transform from base_link to map
    did not become available before timeout
    Failed to bring up all requested nodes. Aborting bringup.

and then every goal is rejected with "Action server is inactive" until the whole
stack is restarted. The SLAM stack used to guard against this with a fixed
``sleep 8`` before starting Nav2, which is a race: FAST-LIO has to load the PCD
map and localize first, and that regularly takes longer than 8 s.

Under sim time there is a second trap. Costmap2DROS measures its activation
timeout as ``now() - start``, and with ``use_sim_time`` and no ``/clock`` yet
``now()`` is 0. The first ``/clock`` then arrives with a large value, so the
elapsed time appears to jump straight past the 60 s limit and activation fails
after ~1.5 s of wall time. Waiting for ``/clock`` here avoids that entirely.

Exit codes: 0 once everything is ready, 1 on timeout (the caller decides whether
to launch anyway).
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
import tf2_ros


class Gate(Node):
    def __init__(self, target, source, use_sim_time):
        super().__init__('wait_for_tf')
        # Set before anything reads the clock, so the buffer agrees with the
        # stack we are gating.
        self.set_parameters(
            [rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)])
        self.target = target
        self.source = source
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.clock_seen = not use_sim_time
        if use_sim_time:
            # /clock is best-effort + volatile from most simulators.
            self.create_subscription(Clock, '/clock', self._on_clock, 10)

    def _on_clock(self, _msg):
        self.clock_seen = True

    def tf_ready(self):
        # Time() is "latest available", so this does not depend on the clock.
        try:
            return self.buffer.can_transform(self.target, self.source, Time())
        except Exception:
            return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('target', nargs='?', default='map')
    ap.add_argument('source', nargs='?', default='base_link')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--sim', action='store_true',
                    help='stack runs with use_sim_time:=true; also wait /clock')
    args = ap.parse_args()

    rclpy.init()
    node = Gate(args.target, args.source, args.sim)
    log = node.get_logger()
    log.info(
        f'waiting for TF {args.target} <- {args.source}'
        + (' and /clock' if args.sim else '')
        + f' (timeout {args.timeout:.0f}s)')

    deadline = time.monotonic() + args.timeout
    rc = 1
    last_note = 0.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.clock_seen and node.tf_ready():
                waited = args.timeout - (deadline - time.monotonic())
                log.info(f'ready after {waited:.1f}s; starting Nav2')
                rc = 0
                break
            now = time.monotonic()
            if now - last_note >= 5.0:
                last_note = now
                log.info(
                    f'still waiting: clock={"ok" if node.clock_seen else "no"} '
                    f'tf={"ok" if node.tf_ready() else "no"} '
                    f'({deadline - now:.0f}s left)')
        else:
            log.error(
                f'timed out after {args.timeout:.0f}s: '
                f'clock={"ok" if node.clock_seen else "MISSING"} '
                f'tf {args.target}<-{args.source}='
                f'{"ok" if node.tf_ready() else "MISSING"}. '
                'Nav2 costmaps will likely fail to activate.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
