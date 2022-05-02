# State Parent Class
# Use this as parent of all states
# Defined by class StateName(State): pass

class State:
    def run(self):
        assert 0, "run not defined"
    
    def next_state(self, input):
        assert 0, "next not defined"

    def prev_state(self):
        assert 0, "no previous state defined"
