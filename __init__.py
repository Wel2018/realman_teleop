"""睿尔曼遥操作客户端"""

from toolbox.qt import qtbase

q_appcfg = qtbase.QAppConfig(
    name = "Realman 遥操作数据采集程序",
    name_en = "Realman Teleop",
    date="2026-01-09",
    version = "0.1.4",
    fontsize = 14,
    slot="realman_teleop",
    APPCFG_DICT=qtbase.get_appcfg(__file__),
)
# logger = get_logger(q_appcfg.slot)

APPCFG = q_appcfg.APPCFG_DICT
ee_type = APPCFG['ee_type']
IS_HAND_MODE = 1 if ee_type == "hand" else 0

API_IP = APPCFG['API_IP']
API_PORT = APPCFG['API_PORT']
ARM_IP = APPCFG['ARM_IP']
API_PRE = f"http://{API_IP}:{API_PORT}"
