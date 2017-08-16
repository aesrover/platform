class Transducer:
    """ A transducer framework class. """
    def __init__(self, atype, itype=None, **other):
        self.data: dict = {'atype': atype, 'itype': itype}
        self.data.update(other)

    def _add_transducer_info(self, data: dict):
        data = dict(data)
        data.update(self.data)
        return data

    def read_transducer(self):
        raise NotImplementedError("Method `read_transducer` not implemented by: {}".format(self))

    def read(self):
        return self._add_transducer_info(self.read_transducer())


class BasicTransducer(Transducer):
    """ A basic transducer sub-class that takes a read function as input. """
    def __init__(self, func, atype, itype=None, **other):
        super().__init__(atype, itype, **other)
        self.read_transducer = func

    def read_transducer(self):
        raise NotImplementedError
