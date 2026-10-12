#!/usr/bin/env python3
"""合并 catkin 各包的编译数据库，供容器内 C/C++ 扩展使用。"""

import json
from pathlib import Path


build = Path('/ws/build')
databases = sorted(build.glob('*/compile_commands.json'))
commands = []
for database in databases:
    commands.extend(json.loads(database.read_text()))

# 完整写入后替换，避免编辑器读到只写了一半的数据库。
temporary = build / 'compile_commands.json.tmp'
temporary.write_text(json.dumps(commands, indent=2) + '\n')
temporary.replace(build / 'compile_commands.json')
print(f'Compilation database: {len(databases)} packages, {len(commands)} commands -> {build}/compile_commands.json')
