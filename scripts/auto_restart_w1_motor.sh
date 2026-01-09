#!/bin/bash

# 发送第一条指令，延迟500ms
cansend can0 033#0402
sleep 0.5

# 发送第二条指令，延迟500ms
cansend can0 033#0502
sleep 0.5

# 发送第三条指令，延迟500ms
cansend can0 033#0602
sleep 0.5

# 发送第四条指令，延迟500ms
cansend can0 033#0401
sleep 0.5

# 发送第五条指令，延迟500ms
cansend can0 033#0501
sleep 0.5

# 发送第六条指令
cansend can0 033#0601
sleep 0.5

