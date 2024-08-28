import os
from functools import partial

from loguru import logger


def update_mesh_dir(old_path, new_directory):
    """
    Updates the file's directory by moving the file from the old path to the new directory,
    keeping the original file name intact.

    Args:
        old_path (str): The current file path.
        new_directory (str): The new directory path where the file should be moved.

    Returns:
        str: The updated file path.

    Raises:
        FileNotFoundError: If the file at the old path does not exist.
        NotADirectoryError: If the new directory path is not a directory.
        Exception: For any other issues that arise during the file move.
    """
    if not os.path.isfile(old_path):
        raise FileNotFoundError(f"The file at {old_path} does not exist.")

    if not os.path.isdir(new_directory):
        raise NotADirectoryError(
            f"The specified path is not a directory: {new_directory}"
        )

    # Get the file name from the old path
    file_name = os.path.basename(old_path)

    # Construct the new path by combining the new directory and the file name
    new_path = os.path.join(new_directory, file_name)

    if os.path.isfile(new_path):
        raise FileExistsError(f"A file already exists at the new path: {new_path}")

    try:
        os.rename(old_path, new_path)
    except Exception as e:
        raise Exception(f"An error occurred while moving the file: {e}")

    return new_path


def filename_handler_null(fname):
    """A lazy filename handler that simply returns its input.

    Args:
        fname (str): A file name.

    Returns:
        str: Same file name.
    """
    return fname


def filename_handler_ignore_directive(fname):
    """A filename handler that removes anything before (and including) '://'.

    Args:
        fname (str): A file name.

    Returns:
        str: The file name without the prefix.
    """
    if "://" in fname or ":\\\\" in fname:
        return ":".join(fname.split(":")[1:])[2:]
    return fname


def filename_handler_ignore_directive_package(fname):
    """A filename handler that removes the 'package://' directive and the package it refers to.
    It subsequently calls filename_handler_ignore_directive, i.e., it removes any other directive.

    Args:
        fname (str): A file name.

    Returns:
        str: The file name without 'package://' and the package name.
    """
    if fname.startswith("package://"):
        string_length = len("package://")
        return os.path.join(
            *os.path.normpath(fname[string_length:]).split(os.path.sep)[1:]
        )
    return filename_handler_ignore_directive(fname)


def filename_handler_add_prefix(fname, prefix):
    """A filename handler that adds a prefix.

    Args:
        fname (str): A file name.
        prefix (str): A prefix.

    Returns:
        str: Prefix plus file name.
    """
    return prefix + fname


def filename_handler_absolute2relative(fname, dir):
    """A filename handler that turns an absolute file name into a relative one.

    Args:
        fname (str): A file name.
        dir (str): A directory.

    Returns:
        str: The file name relative to the directory.
    """
    # TODO: that's not right
    if fname.startswith(dir):
        return fname[len(dir) :]
    return fname


def filename_handler_relative(fname, dir):
    """A filename handler that joins a file name with a directory.

    Args:
        fname (str): A file name.
        dir (str): A directory.

    Returns:
        str: The directory joined with the file name.
    """
    return os.path.join(dir, filename_handler_ignore_directive_package(fname))


def filename_handler_relative_to_urdf_file(fname, urdf_fname):
    return filename_handler_relative(fname, os.path.dirname(urdf_fname))


def filename_handler_relative_to_urdf_file_recursive(fname, urdf_fname, level=0):
    if level == 0:
        return filename_handler_relative_to_urdf_file(fname, urdf_fname)
    return filename_handler_relative_to_urdf_file_recursive(
        fname, os.path.split(urdf_fname)[0], level=level - 1
    )


def _create_filename_handlers_to_urdf_file_recursive(urdf_fname):
    return [
        partial(
            filename_handler_relative_to_urdf_file_recursive,
            urdf_fname=urdf_fname,
            level=i,
        )
        for i in range(len(os.path.normpath(urdf_fname).split(os.path.sep)))
    ]


def filename_handler_meta(fname, filename_handlers):
    """A filename handler that calls other filename handlers until the resulting file name points to an existing file.

    Args:
        fname (str): A file name.
        filename_handlers (list(fn)): A list of function pointers to filename handlers.

    Returns:
        str: The resolved file name that points to an existing file or the input if none of the files exists.
    """
    for fn in filename_handlers:
        candidate_fname = fn(fname=fname)
        logger.debug(f"Checking filename: {candidate_fname}")
        if os.path.isfile(candidate_fname):
            return candidate_fname
    logger.warning(f"Unable to resolve filename: {fname}")
    return fname


def filename_handler_magic(fname, dir):
    """A magic filename handler.

    Args:
        fname (str): A file name.
        dir (str): A directory.

    Returns:
        str: The file name that exists or the input if nothing is found.
    """
    return filename_handler_meta(
        fname=fname,
        filename_handlers=[
            partial(filename_handler_relative, dir=dir),
            filename_handler_ignore_directive,
        ]
        + _create_filename_handlers_to_urdf_file_recursive(urdf_fname=dir),
    )
