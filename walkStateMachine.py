from statemachine import StateMachine, State

class WalkCycleMachine(StateMachine):
    "A walk cycle machine"
    rest = State(initial=True)
    stepping = State()
     
    walk = rest.to(stepping) | stepping.to(rest, cond="step_finished")

    def __init__(self):
        self.active = [None, None, None]
        self.step_finished
        super(WalkCycleMachine, self).__init__()


    def step_finished(self):
        return self.step_finished


if __name__ == '__main__':
    sm = WalkCycleMachine()
    sm._graph().write_png("machine.png")