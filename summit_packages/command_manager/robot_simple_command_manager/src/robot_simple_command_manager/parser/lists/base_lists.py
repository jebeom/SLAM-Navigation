#!/usr/bin/env python
# TODO: These list has not been tested :_D
class StringList(list):
    def __init__(self, iterable):
        super().__init__(str(item) for item in iterable)

    def __setitem__(self, index, item):
        super().__setitem__(index, str(item))

    def insert(self, index, item):
        super().insert(index, str(item))

    def append(self, item):
        super().append(str(item))

    def extend(self, other):
        if isinstance(other, type(self)):
            super().extend(other)
        else:
            super().extend(str(item) for item in other)

class IntList(list):
    def __init__(self, iterable):
        super().__init__(int(item) for item in iterable)

    def __setitem__(self, index, item):
        super().__setitem__(index, int(item))

    def insert(self, index, item):
        super().insert(index, int(item))

    def append(self, item):
        super().append(int(item))

    def extend(self, other):
        if isinstance(other, type(self)):
            super().extend(other)
        else:
            super().extend(int(item) for item in other)

class FloatList(list):
    def __init__(self, iterable):
        super().__init__(float(item) for item in iterable)

    def __setitem__(self, index, item):
        super().__setitem__(index, float(item))

    def insert(self, index, item):
        super().insert(index, float(item))

    def append(self, item):
        super().append(float(item))

    def extend(self, other):
        if isinstance(other, type(self)):
            super().extend(other)
        else:
            super().extend(float(item) for item in other)

class BoolList(list):
    def __init__(self, iterable):
        super().__init__(bool(item) for item in iterable)

    def __setitem__(self, index, item):
        super().__setitem__(index, bool(item))

    def insert(self, index, item):
        super().insert(index, bool(item))

    def append(self, item):
        super().append(bool(item))

    def extend(self, other):
        if isinstance(other, type(self)):
            super().extend(other)
        else:
            super().extend(bool(item) for item in other)
