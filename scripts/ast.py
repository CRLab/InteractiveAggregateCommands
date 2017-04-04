class Task: pass

class GoTo(Task):
    def __init__(self,location):
        self.type = "GoTo"
        self.location = location

