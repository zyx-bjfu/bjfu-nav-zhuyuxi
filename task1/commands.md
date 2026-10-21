# 常用终端命令记录（task1 - Hello Ubuntu & Git）

## 一、目录与文件操作
| 命令 | 作用 | 示例 |
|------|------|------|
| pwd | 显示当前所在目录 | pwd -> /home/zyx |
| ls | 列出当前目录文件 | ls |
| ls -l | 详细列表（含权限）| ls -l hello.txt |
| cd <目录> | 进入目录 | cd mylab |
| mkdir <名字> | 新建文件夹 | mkdir mylab |
| touch <文件> | 新建空文件 | touch hello.txt |
| cp <源> <目标> | 复制文件 | cp hello.txt hello2.txt |
| mv <源> <目标> | 移动/重命名 | mv hello2.txt mynotes.txt |
| rm <文件> | 删除文件（不可恢复！）| rm mynotes.txt |

## 二、权限操作
| 命令 | 作用 |
|------|------|
| chmod 777 <文件> | 所有人可读/写/执行（rwxrwxrwx）|
| chmod 644 <文件> | 拥有者读写、其他人只读（rw-r--r--）|

权限数字记忆：读r=4、写w=2、执行x=1，三位数字分别代表 拥有者/同组/其他人。

## 三、系统与软件
| 命令 | 作用 |
|------|------|
| sudo <命令> | 以管理员权限执行 |
| sudo apt update | 刷新软件列表 |
| sudo apt install <软件> | 安装软件 |

## 四、备注（踩坑记录）
- Linux 命令严格区分大小写：ls 有效，Ls 无效
- 输入 sudo 密码时屏幕无显示，是正常现象，盲打后回车
- 终端粘贴快捷键：Ctrl+Shift+V
- ls -l 是字母 l（list），不是数字 1
- rm 删除的文件不进回收站，用前确认文件名