# Git 空间回收与编辑器索引缓存

2026-10-25 已使用 Git 原生 `prune` 回收超过 24 小时且不受任何引用或 reflog 保留的旧松散对象。未执行 reflog 过期、引用删除、reset、clean；未修改聊天记录、代码、模型、标定数据和已安装功能包。

清理前后审计见 `git_safe_reclaim_20261025.json`。清理前的分类审计 `git_objects_deep_audit_20261025.json` 属于历史记录，不代表当前大小。

## 已实施的防膨胀设置

- 根目录 `.gitignore` 排除 `browse.vc.db` 及其旁文件、`*.ipch` 和 `.vscode/ipch/`，同时保留编辑器配置文件。
- 根目录及 `piper_ros` 的 `.vscode/c_cpp_properties.json` 将 C/C++ 浏览数据库分别改到 `$HOME/.cache/vscode-cpptools/GraspGen/` 和 `$HOME/.cache/vscode-cpptools/GraspGen-piper_ros/`，离开项目目录。
- 在 VS Code 命令面板执行 **Developer: Reload Window（开发人员：重新加载窗口）**，让 C/C++ 扩展重新加载配置。现有项目内数据库可能仍被当前编辑器使用，本次没有强行删除。

忽略规则不追溯改写已有快照，也不保证 Git 永远不增长；正常代码历史和仍被引用的快照会继续保留。不要通过删除整个 `.git` 或 `objects` 来清理。

## 后续检查

在项目目录运行（只读）：

```bash
git count-objects -vH
du -sh .git
git check-ignore .vscode/browse.vc.db piper_ros/.vscode/browse.vc.db
```

今天写入的无引用打包对象本次仍保留，避免与近期快照并发写入冲突。没有设置无人值守的强制清理任务，也没有缩短 Git 默认自动 GC 的宽限期。需要进一步回收打包对象时，应先等其超过宽限期，确认没有其他 Git 写入，保存引用清单，再由 Git 原生 repack/GC 完成并验证；不要手动删除 pack 文件或使用 `--prune=now`。
