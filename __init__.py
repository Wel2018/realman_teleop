"""睿尔曼遥操作客户端"""

from toolbox.qt import qtbase
from .version import __version__
from .version import __update_timestamp__


q_appcfg = qtbase.QAppConfig(
    name = "Realman 遥操作数据采集程序",
    name_en = "Realman Teleop",
    date = __update_timestamp__,
    version = __version__,
    fontsize = 14,
    slot="realman_teleop",
    APPCFG_DICT=qtbase.get_appcfg(__file__),
)

APPCFG = q_appcfg.APPCFG_DICT
API_IP = APPCFG['API_IP']
API_PORT = APPCFG['API_PORT']
API_PRE = f"http://{API_IP}:{API_PORT}"
