@echo off
chcp 65001 >nul
title 生化培养箱上位机
echo ============================================
echo   生化培养箱控制系统 — 上位机
echo ============================================
echo.

:: 检查 Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [错误] 未检测到 Python，请先安装 Python 3.8+
    echo 下载地址: https://www.python.org/downloads/
    pause
    exit /b 1
)

:: 安装依赖
echo 正在检查依赖...
pip install -r "%~dp0requirements.txt" -q
if %errorlevel% neq 0 (
    echo [警告] 部分依赖安装失败，尝试使用清华镜像...
    pip install -r "%~dp0requirements.txt" -q -i https://pypi.tuna.tsinghua.edu.cn/simple
)

echo.
echo 启动上位机...
echo.
python "%~dp0main.py"
pause
