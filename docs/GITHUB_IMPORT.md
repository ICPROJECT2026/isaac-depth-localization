# 导入 IC380PROJECT206

建议目标为 `IC380PROJECT206/isaac-depth-localization`。本准备工作只生成本地交付文件，不自动创建组织仓库、上传资源或推送代码。

## 1. 确认上传目录

请进入整理版 `isaac-depth-localization` 目录，不要对原来包含约 5.4 GB 输出的工作目录执行 `git add .`。

```bash
python3 scripts/check-repository.py
git status --short
```

若是从不带 `.git` 的代码压缩包解压：

```bash
git init -b main
```

大资源、`.env.local`、本地生成配置、bag、编译工作空间和日志都已被忽略。检查 `git status` 后再提交：

```bash
git add .
git diff --cached --stat
git commit -m "Prepare Isaac Sim depth localization project"
```

提交需要使用操作者自己的 Git 身份；本准备包没有设置虚假的作者邮箱或替组织选择公开许可证。

## 2. 在组织下创建空仓库

登录有权限的 GitHub 账号，创建仓库时 owner 选择 **IC380PROJECT206**。名称建议 `isaac-depth-localization`；可见性按组织要求选择。不要在 GitHub 自动生成 README、gitignore 或 License，以免与本地初始提交冲突。

复制新仓库的地址后执行：

```bash
git remote add origin https://github.com/IC380PROJECT206/isaac-depth-localization.git
git remote -v
git push -u origin main
```

如果已有 `origin`，先用 `git remote -v` 检查，不要盲目覆盖。若组织已有非空仓库，应先克隆该仓库，在新分支中复制本准备包并提 PR，不要强推覆盖其历史。

也可以使用已认证的 GitHub CLI 创建**私有**仓库并推送，选择该方式时不要再重复网页创建：

```bash
gh auth status
gh repo create IC380PROJECT206/isaac-depth-localization --private --source=. --remote=origin --push
```

上述命令是供维护者执行的发布步骤，不会被本仓库任何启动脚本自动调用。参考 [GitHub 官方导入文档](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)。

## 3. 本地配置资源

仅推送代码和配置，不上传场景、PCD 或独立资源压缩包。其他成员需要自行准备本地场景与地图，按 [`ASSETS.md`](ASSETS.md) 指定资源路径。

当前辅助栅格 `room.yaml` / `room.pgm` 是小型搜索配置数据，与历史房间 PCD 配套；更换地图时必须同步更换。大 PCD、USD 几何、贴图和录制文件已由 `.gitignore` 排除。

## 建议仓库信息

- Description：`RGB-D point-cloud localization on a prior 3D map with wheel odometry in Isaac Sim and ROS 2 Humble.`
- Topics：`isaac-sim`、`ros2`、`humble`、`localization`、`point-cloud`、`ndt`、`rgbd`、`wheel-odometry`。
- 主分支：`main`；静态检查由 `.github/workflows/check.yml` 执行，不要求 GPU。
- 开源许可证：由组织决定后新增 `LICENSE` 并同步 README。第三方依赖和场景素材仍保留各自许可。
