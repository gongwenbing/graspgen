"""Bounded exact pairing: IDs are local to a single stamped frame."""
from collections import OrderedDict


class FramePairer:
    def __init__(self, capacity=5):
        self.capacity = capacity
        self.clouds = OrderedDict()
        self.metadata = OrderedDict()
        self.last_processed = -1

    def put(self, kind, key, value):
        frame, sec, nsec = key
        stamp = sec * 10**9 + nsec
        if not frame or not 0 <= nsec < 10**9 or sec <= 0:
            raise ValueError('Invalid frame header')
        if stamp <= self.last_processed:
            return None
        cache = self.clouds if kind == 'cloud' else self.metadata
        cache[key] = value
        while len(cache) > self.capacity:
            cache.popitem(last=False)
        if key not in self.clouds or key not in self.metadata:
            return None
        result = self.clouds.pop(key), self.metadata.pop(key)
        self.last_processed = stamp
        for cache in (self.clouds, self.metadata):
            for old in list(cache):
                if old[1] * 10**9 + old[2] <= stamp:
                    del cache[old]
        return result
