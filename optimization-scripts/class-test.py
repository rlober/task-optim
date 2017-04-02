
class Base(object):
    """docstring for Base."""
    def __init__(self):
        pass

    def pp(self):
        print("Base")

    def doStuff(self):
        self.pp()


class Derived(Base):
    """docstring for Derived."""
    def __init__(self):

        super(Derived, self).__init__()

    def pp(self):
        print("Derived")



if __name__ == "__main__":

    b = Base()
    d = Derived()

    print("b.doStuff()")
    b.doStuff()
    print("d.doStuff()")
    d.doStuff()
