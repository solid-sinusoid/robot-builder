import numpy as np


def array_eq(arr1, arr2):
    if arr1 is None and arr2 is None:
        return True
    return (
        isinstance(arr1, np.ndarray)
        and isinstance(arr2, np.ndarray)
        and arr1.shape == arr2.shape
        and (arr1 == arr2).all()
    )


def str2float(s):
    """Cast string to float if it is not None. Otherwise return None.

    Args:
        s (str): String to convert or None.

    Returns:
        str or NoneType: The converted string or None.
    """
    return float(s) if s is not None else None
