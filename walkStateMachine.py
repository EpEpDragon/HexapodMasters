from statemachine import StateMachine, State
from pyray import *

rest_pos = [Vector2(86.6, -50.0), Vector2(86.6, 50.0),
                Vector2(0, -100.0), Vector2(0, 100.0),
                Vector2(-86.6, -50.0), Vector2(-86.6, 50.0)]

stride_length = 30

def vector2_close(a,b, close):
    diff_x = abs(a.x - b.x)
    diff_y = abs(a.y - b.y)
    if diff_x < close and diff_y < close:
        return True
    return False

class WalkCycleMachine(StateMachine):
    "A walk cycle machine"
    rest = State(initial=True)
    stepping = State()
     
    walk = rest.to(stepping, cond="has_speed") | stepping.to(rest, cond="step_finished") | rest.to.itself(internal=True) | stepping.to.itself(internal=True) 

    def __init__(self):
        self.active = [None, None, None]
        self.inactive = [None, None, None]
        self.walk_direction = Vector2(0,0)
        self.foot_pos = [Vector2(86.6, -50.0), Vector2(86.6, 50.0),
                Vector2(0, -100.0), Vector2(0, 100.0),
                Vector2(-86.6, -50.0), Vector2(-86.6, 50.0)]
        self.targets = [Vector2(86.6, -50.0), Vector2(86.6, 50.0),
                Vector2(0, -100.0), Vector2(0, 100.0),
                Vector2(-86.6, -50.0), Vector2(-86.6, 50.0)]
        
        super(WalkCycleMachine, self).__init__()

    def on_enter_stepping(self):
        self.update_targets()

    def step_finished(self):
        for i in range(6):
            if not vector2_close(self.foot_pos[i], self.targets[i], 5):
                return False
        return True
    
    def has_speed(self):
        return not vector2_equals(self.walk_direction, Vector2(0,0))

    def update_targets(self):
        for i in self.active:
            if i is not None:
                self.targets[i] = vector2_add(rest_pos[i], vector2_scale(self.walk_direction,stride_length))
        for i in self.inactive:
            if i is not None:
                self.targets[i] = vector2_subtract(rest_pos[i], vector2_scale(self.walk_direction,stride_length))



if __name__ == '__main__':
    sm = WalkCycleMachine()
    sm._graph().write_png("machine.png")