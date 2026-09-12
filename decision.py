from config import CENTER_TOLERANCE, TARGET_AREA_NEAR


class DecisionMaker:
    def __init__(self, frame_width):
        self.frame_width = frame_width
        self.frame_center_x = frame_width // 2

    def decide(self, target):
        if target is None:
            return "SEARCH"

        target_center_x = target["x"] + target["w"] // 2
        target_area = target["area"]

        offset = target_center_x - self.frame_center_x

        if target_area >= TARGET_AREA_NEAR:
            return "COLLECT"

        if offset < -CENTER_TOLERANCE:
            return "TURN_LEFT"

        if offset > CENTER_TOLERANCE:
            return "TURN_RIGHT"

        return "MOVE_FORWARD"
