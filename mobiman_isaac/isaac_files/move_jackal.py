from omni.kit.scripting import BehaviorScript


class MoveJackal(BehaviorScript):
    def on_init(self):
        print(f"{__class__.__name__}.on_init()->{self.prim_path}")

    def on_destroy(self):
        print(f"{__class__.__name__}.on_destroy()->{self.prim_path}")

    def on_play(self):
        print(f"{__class__.__name__}.on_play()->{self.prim_path}")

    def on_pause(self):
        print(f"{__class__.__name__}.on_pause()->{self.prim_path}")

    def on_stop(self):
        print(f"{__class__.__name__}.on_stop()->{self.prim_path}")

    def on_update(self, current_time: float, delta_time: float):
        print(f"{__class__.__name__}.on_update(current_time={current_time}, delta_time={delta_time})->{self.prim_path}")
