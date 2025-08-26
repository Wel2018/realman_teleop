"""睿尔曼模仿学习客户端"""

from toolbox.qt import qtbase
from toolbox.core.logbase import get_logger


q_appcfg = qtbase.QAppConfig(
    name = "Realman 遥操作数据采集程序",
    name_en = "Realman Teleop",
    date="2025-08-26",
    version = "1.0.0",
    fontsize = 14,
    slot="realman_teleop",
    APPCFG_DICT=qtbase.get_appcfg(__file__),
)

print(f"q_appcfg={q_appcfg}")
logger = get_logger(q_appcfg.slot)
