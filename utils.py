import functools
import time
from typing import Callable


def time_it(func: Callable):

    @functools.wraps(func)
    def time_it_wrapper(*args, **kwargs):

        start = time.perf_counter()
        value = func(*args, **kwargs)
        stop = time.perf_counter()
        print(f"{func.__name__} finished in {stop - start} s.")

        return value

    return time_it_wrapper
