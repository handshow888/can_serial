#!/bin/bash

# Shell 脚本：设置 CAN 接口自启动，并将当前用户添加到 dialout 组以赋予永久串口权限

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then
    echo "请以 root 用户权限运行此脚本（使用 sudo）"
    exit 1
fi

# 获取当前用户名
CURRENT_USER=$(logname)
echo "当前用户: $CURRENT_USER"

# 定义服务名称和服务文件路径
SERVICE_NAME="can-setup.service"
SERVICE_FILE="/etc/systemd/system/$SERVICE_NAME"

# 创建 systemd 服务文件
echo "正在创建 systemd 服务文件: $SERVICE_FILE"
cat <<EOF | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=Setup CAN interface can0 at boot
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -c "while true; do if ip link show can0 >/dev/null 2>&1; then echo '找到 can0 设备，正在尝试启动...'; if ip link set can0 type can bitrate 1000000 loopback off && ip link set can0 up; then echo 'can0 成功启动！'; break; else echo '启动 can0 失败，将在0.5秒后重试...'; sleep 0.5; fi; else echo '等待 can0 设备出现...'; sleep 0.5; fi; done"
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# 设置服务文件权限
echo "设置服务文件权限..."
sudo chmod 644 "$SERVICE_FILE"

# 重新加载 systemd 配置
echo "重新加载 systemd 配置..."
sudo systemctl daemon-reload

# 启用服务以在启动时运行
echo "启用服务以在启动时运行..."
sudo systemctl enable "$SERVICE_NAME"

# 启动服务并检查状态（可选）
echo "启动服务并检查状态..."
sudo systemctl start "$SERVICE_NAME"
sudo systemctl status "$SERVICE_NAME" --no-pager

# 将当前用户添加到 dialout 组以赋予永久串口权限
echo "将当前用户 ($CURRENT_USER) 添加到 dialout 组..."
if id -nG "$CURRENT_USER" | grep -qw "dialout"; then
    echo "用户 $CURRENT_USER 已经在 dialout 组中，无需重复添加。"
else
    sudo usermod -aG dialout "$CURRENT_USER"
    echo "用户 $CURRENT_USER 已成功添加到 dialout 组。"
    echo "注意：需要重新登录或重启系统才能使更改生效。"
fi

# 提示完成
echo "CAN 自启动配置和串口权限配置已完成！"
echo "系统将在下次启动时自动启用 can0 接口。"