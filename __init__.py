"""睿尔曼遥操作客户端"""

from toolbox.core.color_print import printc
from toolbox.qt import qtbase
from toolbox.core.logbase import get_logger


q_appcfg = qtbase.QAppConfig(
    name = "Realman 遥操作数据采集程序",
    name_en = "Realman Teleop",
    date="2026-01-09",
    version = "0.1.3",
    fontsize = 14,
    slot="realman_teleop",
    APPCFG_DICT=qtbase.get_appcfg(__file__),
)

printc(f"q_appcfg={q_appcfg}")
logger = get_logger(q_appcfg.slot)
