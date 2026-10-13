from config import CENTER_TOLERANCE, TARGET_AREA_NEAR


class DecisionMaker:
    def __init__(self, frame_width):
        self.frame_width = frame_width
        self.frame_center_x = frame_width // 2
        # 记录最后一次看到球时的转向方向，用于丢目标后继续朝该方向搜索
        self.last_seen_direction = "TURN_LEFT"

    def decide(self, target):
        if target is None:
            # 丢目标：沿最后看到球的方向继续转，而不是固定左转
            return self.last_seen_direction

        target_center_x = target["x"] + target["w"] // 2
        target_area = target["area"]

        offset = target_center_x - self.frame_center_x

        if target_area >= TARGET_AREA_NEAR:
            return "COLLECT"

        if offset < -CENTER_TOLERANCE:
            direction = "TURN_LEFT"
        elif offset > CENTER_TOLERANCE:
            direction = "TURN_RIGHT"
        else:
            return "MOVE_FORWARD"

        # 只要还能看到球，就更新搜索方向
        self.last_seen_direction = direction
        return direction
