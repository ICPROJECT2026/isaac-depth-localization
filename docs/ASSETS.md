# 场景与地图资源

Git 仓库保存代码、模板、参数、辅助栅格和小型结果图。完整 collected 场景和点云作为独立资源交付。

## 恢复目录

```text
assets/
├── manifest.json
├── maps/
│   ├── room.pcd
│   ├── room.yaml
│   └── room.pgm
└── scene/
    └── Collected_final_wheeltec_with_scene/
        ├── final_wheeltec_with_scene.usd
        └── ... 全部原始相对目录与资源
```

`room.pcd` 是原 `房间范围_2cm_去顶板查看版.pcd` 的交付名称，不改变点集。11,438,552 点、183,017,078 字节。PCD 单位为米，Z 向上；地图原点、地面高度、辅助栅格和场景覆盖层必须一致。

## 使用本机资源

本项目不打包或分发原始场景与点云。可以直接在 README 的 configure 命令中传入本机已有文件，不需要放入仓库，也不需要上传 GitHub。

`assets/manifest.json` 仅保留历史实验资源的相对名称、字节数和 SHA-256，便于确认实验资源版本；它不包含场景或点云数据。如果本机有同一套历史资源，可选执行：

```bash
python3 scripts/verify-assets.py \
  --scene-dir /path/to/Collected_final_wheeltec_with_scene \
  --map /path/to/room.pcd
```

如果使用其他资源，不要求匹配此历史清单，但需要同步调整场景接口与配套搜索栅格。原始场景根文件、引用的材质与贴图必须保持完整目录结构。

## Git 上传范围

只上传代码、配置、辅助搜索栅格和小型实验结果。PCD、PLY、原始 USD、录制文件和本机资源目录已通过 `.gitignore` 排除，不需要配置 Git LFS 或上传独立资源压缩包。

`room.yaml` / `room.pgm` 是用于 BBS 初始化的小型辅助搜索栅格，不是完整三维 PCD。它们只与历史房间地图配套。

## 更换场景或地图

1. 同步更新 USD 覆盖层中的 prim 路径、比例和相机/关节接口。
2. 准备单位和原点一致的 PCD，保留场景地面 Z 高度约定。
3. 用上游提供的 `generate_occupancy_map_from_pcd` 工具按对应版本的帮助生成配套二维搜索栅格；本项目原参数为 0.05 m、离地 0.2–1.7 m、每格至少 2 点、不膨胀、外扩 0.5 m。
4. 更新全局搜索高度先验等场景参数，传入 `configure.py --occupancy`，再做独立验证。

切换地图不是单纯改文件名，尤其不要把原房间的 `room.yaml` 与新房间的 PCD 混用。
