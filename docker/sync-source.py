#!/usr/bin/env python3
"""Stage Linux sources without macOS AppleDouble files or imported build caches."""
# 将主机只读源码挂载同步进容器的持久 source 卷，供 ROS/catkin 在 Linux 环境编译。
from pathlib import Path
import shutil

source = Path('/host-src')
target = Path('/ws/src')
# 防止路径挂载错误时误删普通目录内容。
if not source.is_dir() or not target.is_mount():
    raise SystemExit('Expected /host-src bind mount and /ws/src source volume')

def ignored(name):
    # 排除 macOS AppleDouble 元数据和不应导入的本机 CMake 构建目录。
    return name.startswith('._') or name in {'.DS_Store', 'cmake-build-debug'}

def sync(src, dst):
    # 递归对齐目录内容：源端已删除的项目会从目标卷删除，新增/变更文件会复制。
    dst.mkdir(parents=True, exist_ok=True)
    names = {p.name for p in src.iterdir() if not ignored(p.name)}
    for old in dst.iterdir():
        if old.name not in names:
            if old.is_dir() and not old.is_symlink():
                shutil.rmtree(old)
            else:
                old.unlink()
    for name in names:
        a, b = src / name, dst / name
        if a.is_dir():
            # 目录递归同步，文件则按大小和修改时间判断是否需要复制。
            sync(a, b)
        elif not b.exists() or (a.stat().st_size, a.stat().st_mtime_ns) != (b.stat().st_size, b.stat().st_mtime_ns):
            shutil.copy2(a, b)

# 同步完成后打印提示；编译仍由 build.sh 执行。
sync(source, target)
print('Linux source volume synchronized (AppleDouble/build caches excluded).')
