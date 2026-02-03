from collections import deque


class SpaceMouseButtonClickDetector:
    """
    基于 deque 的单击/双击检测。
    每次完整的 press->release（1->0）视为一次点击。
    """
    def __init__(self, name: str, double_interval_ms: int = 250):
        self.name = name
        self.double_interval = double_interval_ms / 1000.0
        self.click_times = deque()   # 存放每次完整点击的时间戳
        self.last_raw = 0            # 上一次原始值（用于检测 1->0）
        self.pending_single = None   # 记录等待确认的单击事件时间戳

    def update(self, raw: int, now: float) -> dict | None:
        """
        raw: 当前按钮状态 (0/1)
        now: 当前时间戳（秒）
        """
        event = None

        # 检测一次完整点击：press(1) -> release(0)
        if self.last_raw == 1 and raw == 0:
            self.click_times.append(now)

        self.last_raw = raw

        # —— 无点击则不用处理 ——
        if not self.click_times:
            return None

        # —— 处理点击队列 ——
        while len(self.click_times) >= 2:
            t1 = self.click_times[0]
            t2 = self.click_times[1]

            if t2 - t1 <= self.double_interval:
                # 两次点击在间隔内 → 双击
                self.click_times.clear()
                return {'type': 'double', 'btn': self.name}
            else:
                # 间隔太长 → t1 必然是单击
                self.click_times.popleft()
                return {'type': 'single', 'btn': self.name}

        # —— 只有一次点击，需要等待查看是否会出现第二次 ——
        t1 = self.click_times[0]

        if now - t1 > self.double_interval:
            # 超时 → 单击成立
            self.click_times.clear()
            return {'type': 'single', 'btn': self.name}

        return None

