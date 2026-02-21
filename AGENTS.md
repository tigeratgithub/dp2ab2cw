# Repository Guidelines

## 项目结构与模块组织
- `src/`：应用源代码。核心文件包括 `qeo_abz.c`（QEO ABZ 生成逻辑）与 `mmc.c`。
- `doc/`：文档资源（如波形图片）。
- `build/`：CMake 构建输出（生成目录）。
- `hpm5361cf_flash_xip_debug/`、`segger_prj/`：IDE/调试产物（生成目录）。
- 顶层文档：`README_zh.rst`、`README_en.rst`、`index_zh.rst`。

## 构建、测试与开发命令
本项目使用 HPMicro SDK 的 CMake 预设。典型流程：

```powershell
# 配置（Debug）
cmake --preset "flash_xip Debug"

# 构建
cmake --build build

# 清理（删除构建输出）
Remove-Item -Recurse -Force build
```

预设与工具链路径定义在 `CMakePresets.json`。项目要求 `HPM_SDK_BASE` 与 GNU RISC-V 工具链可用。

## 编码风格与命名规范
- 语言：C。
- 缩进：4 空格（与现有文件一致）。
- 命名：函数/变量用 `snake_case`，宏与常量用 `UPPER_SNAKE_CASE`。
- 驱动调用与寄存器访问建议分组并添加注释，便于维护。

## 测试指南
当前无自动化测试框架，验证以硬件为主：
- 下载固件后，用示波器/逻辑分析仪观察 ABZ 波形。
- 查看串口输出的状态信息（见 `README_zh.rst`）。

## 提交与合并请求规范
现有 Git 历史较少，未形成固定规范。建议使用简短的祈使句摘要，例如：
- `Add QEO ABZ demo`
- `Fix TRGM routing`

提交 PR 时请包含：
- 行为变更的简要描述。
- 使用的板卡/SDK 版本。
- 若影响信号行为，提供证据（串口日志片段或波形截图）。

## 配置与硬件说明
- 板卡与构建类型通过 CMake 预设设置（`BOARD=hpm5361cf`、`HPM_BUILD_TYPE=flash_xip`）。
- QEO/MMC/QEIV2 的路由依赖 `qeo_abz.c` 中的 TRGM 配置；修改引脚复用时保持一致。
