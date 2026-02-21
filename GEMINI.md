# dp2ab2cw - HPM5361 QEO ABZ 编码器示例

## 项目概述

本项目是 HPM5361 微控制器的示例应用程序，演示了 **QEO (正交编码器输出)** 外设的使用。它展示了如何根据位置信息生成增量式 ABZ 正交编码器信号。

应用程序支持两种操作模式：
1.  **软件位置注入 (Software Position Injection):** CPU 将位置值注入 QEO 外设以模拟编码器运动。
2.  **硬件位置输入 (Hardware Position Input):** **MMC (电机运动控制)** 外设通过 TRGM (触发器多路复用器) 向 QEO 提供位置信息，从而实现实时的硬件驱动信号生成。

## 主要功能

*   **QEO 配置:** 设置分辨率线数、最大频率和信号类型。
*   **QEI v2 使用:** 配置正交编码器接口 (版本 2) 以读取和验证生成的信号。
*   **MMC 集成:** 使用电机运动控制模块在开环模式下模拟电机/编码器位置源。
*   **信号输出:** 生成 A、B 和 Z 信号，其中 A/B 是 90 度相移的正交信号，Z 是每转一个的索引脉冲。

## 目录结构

*   `src/`: 源代码目录。
    *   `qeo_abz.c`: 主应用程序入口点。包含 QEO、QEI 的初始化逻辑和主循环。
    *   `mmc.c`: 用于生成位置数据的 MMC 外设配置。
    *   `moto.h`: 电机/运动控制相关定义的头文件。
*   `CMakeLists.txt`: CMake 构建脚本，链接 `hpm-sdk`。
*   `app.yaml`: 应用程序元数据和依赖项 (`qeo`, `mmc`)。
*   `README_en.rst`: 示例功能的详细文档。

## 构建与运行

本项目使用 **CMake** 并依赖 **HPM SDK**。

### 先决条件

*   必须安装 **HPM SDK** 并设置 `HPM_SDK_BASE` 环境变量。
*   **工具链:** RISC-V GCC 工具链 (或其他支持的工具链，如 IAR/Segger)。

### 构建

1.  **生成构建系统:**
    ```bash
    cmake -S . -B build -GNinja
    ```
    *(注: 根据需要调整 `-G` 生成器，例如如果 Ninja 不可用，则使用 "MinGW Makefiles" 或 "Unix Makefiles")。*

2.  **编译:**
    ```bash
    cmake --build build
    ```

### 烧录和调试

*   该项目分别在 `segger_prj` 和 `iar_embedded_workbench` 目录中提供了 **Segger Embedded Studio** 和 **IAR Embedded Workbench** 的配置。
*   输出二进制文件 (ELF, binary) 位于 `build/output/` 中。
*   使用 JTAG/SWD 调试器 (如 J-Link 或 DAPLink) 将二进制文件烧录到 HPM5361 板上。

## 开发规范

*   **编码风格:** 遵循 HPM SDK C 编码标准。
*   **日志记录:** 使用标准 `printf` 将调试输出发送到串行控制台。
*   **硬件抽象:** 严重依赖 SDK 驱动程序 (`hpm_*_drv.h`) 进行外设访问。
*   **引脚配置:** 引脚复用在 `main` 或板级初始化文件 (`board_init`) 中处理。

## 关键 API 和外设

*   **QEO (正交编码器输出):** `hpm_qeo_drv.h`
*   **QEIv2 (正交编码器接口):** `hpm_qeiv2_drv.h`
*   **MMC (电机运动控制):** `hpm_mmc_drv.h`
*   **TRGM (触发器多路复用器):** `hpm_trgm_soc_drv.h`
