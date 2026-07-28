#!/usr/bin/env python3
"""极简 NaviSkill WebSocket RPC 客户端（供 shell 脚本调用单个 RPC）。

协议同 server.py 的 /ws：请求 {"id","method","params"}，回复 {"id","result"} 或
{"id","error"}。仅做「发一个方法、拿一次结果、打印、退出」，方便 switch_nav_mode.sh
等脚本对接（如 set_nav_mode / get_status / get_nav_mode）。

用法:
  navi_rpc.py <method> [json-params] [--ws URL] [--field KEY] [--timeout SEC]

示例:
  navi_rpc.py set_nav_mode '{"mode":"gps"}'
  navi_rpc.py get_status --field localized
  navi_rpc.py get_nav_mode --ws ws://127.0.0.1:9000/ws

退出码:
  0 成功（result.success 非 False）
  2 参数 JSON 非法
  3 连接 / 传输失败
  4 服务端返回 error
  5 result.success 为 False
"""

import argparse
import asyncio
import json
import sys

import websockets


async def _call(ws_url: str, method: str, params: dict, timeout: float) -> dict:
    ws = await asyncio.wait_for(websockets.connect(ws_url), timeout=timeout)
    try:
        await ws.send(json.dumps({"id": 1, "method": method, "params": params},
                                 ensure_ascii=False))
        raw = await asyncio.wait_for(ws.recv(), timeout=timeout)
        return json.loads(raw)
    finally:
        try:
            await ws.close()
        except Exception:
            pass


def main() -> None:
    p = argparse.ArgumentParser(description="NaviSkill 单次 WebSocket RPC 调用")
    p.add_argument("method", help="RPC 方法名，如 set_nav_mode / get_status")
    p.add_argument("params", nargs="?", default="{}",
                   help="JSON 参数（默认 {}）")
    p.add_argument("--ws", default="ws://127.0.0.1:9000/ws",
                   help="WebSocket 地址（默认 ws://127.0.0.1:9000/ws；测试端口 9999）")
    p.add_argument("--field", default=None,
                   help="只打印 result 中该字段（便于 shell 取值）")
    p.add_argument("--timeout", type=float, default=10.0, help="超时秒数")
    a = p.parse_args()

    try:
        params = json.loads(a.params)
        if not isinstance(params, dict):
            raise ValueError("params 必须是 JSON 对象")
    except (json.JSONDecodeError, ValueError) as e:
        print(f"invalid params json: {e}", file=sys.stderr)
        sys.exit(2)

    try:
        msg = asyncio.run(_call(a.ws, a.method, params, a.timeout))
    except Exception as e:  # 连接 / 超时 / 传输
        print(f"rpc error: {e}", file=sys.stderr)
        sys.exit(3)

    if isinstance(msg, dict) and msg.get("error"):
        print(json.dumps(msg, ensure_ascii=False), file=sys.stderr)
        sys.exit(4)

    result = msg.get("result") if isinstance(msg, dict) else None
    if a.field is not None:
        val = result.get(a.field) if isinstance(result, dict) else None
        print(val if isinstance(val, str) else json.dumps(val, ensure_ascii=False))
    else:
        print(json.dumps(result, ensure_ascii=False))

    if isinstance(result, dict) and result.get("success") is False:
        sys.exit(5)


if __name__ == "__main__":
    main()
