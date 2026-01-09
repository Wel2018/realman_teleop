import os
import sys
from loguru import logger

# 1. 配置 Loguru
# 移除默认配置
logger.remove()

# format = (
#     "<green>{time:HH:mm:ss.SSS}</green> "
#     "[<level>{level: <8}</level>] "
#     "<cyan>{module}:{function}:{line}</cyan>"
#     " - "
#     "<level>{message}</level>")

# 核心变量说明：
# {name}: 调用日志的模块名（文件名）
# {function}: 调用日志的函数名
# {line}: 调用日志的行号
# {file}: 完整的文件路径（和 name 不同，name 是模块名，file 是全路径）
# {module}: 和 name 等价，模块名

# 确保日志目录存在
log_dir = "./logs"
os.makedirs(log_dir, exist_ok=True)

cmd_format = (
    "<magenta>{time:HH:mm:ss}</magenta> "
    "🟦 [<cyan>{file}</cyan>, "
    "<cyan>{line}</cyan>] "
    "[<yellow>{name}:{function}</yellow>] "
    "<bold><blue>{message}</blue></bold>"
)

file_format = (
    "{time:YYYY-MM-DD HH:mm:ss} | "
    "{level: <8} |"
    "{name}:{function}:{line} - {message}"
)


def init_logger():
    # 添加控制台输出 (可以保留你的特殊颜色格式)
    logger.add(
        sys.stdout, 
        format=cmd_format,
        colorize=True
    )

    # 添加文件输出 (自动过滤 ANSI 代码，并按天滚动)
    logger.add(
        # 日志文件名格式：包含日期，方便识别
        os.path.join(log_dir, "realman_teleop_{time:YYYY-MM-DD}.log"),
        # 核心：按天轮转（每天 00:00 切换）
        rotation="00:00",
        # 可选：保留最近 7 天的日志，自动删除更早的
        retention="7 days",
        # 可选：过期日志自动压缩为 zip 格式，节省空间
        compression="zip",
        # 包含调用者信息（结合你上一个问题的需求）
        format=file_format,
        # 编码格式，避免中文乱码
        encoding="utf-8",
        # 异步写入，不阻塞主程序
        enqueue=True
    )
    return logger


def printc(msg: str):
    """
    使用 Loguru 实现的日志打印
    """
    logger.opt(depth=1).info(msg)
