import json
import os


def smartload(x) -> dict:
    """
    Parse a JSON string or file and return a dict.

    Interpret the input as either a JSON str or a JSON file in a
    robust manner, and give useful errors otherwise.

    args:
    x should be a string containing valid JSON, or a file descriptor (path string).

    NB: Interpreting x as a file takes precedence over interpreting x
        as a JSON str. If x is '{"a": null}' and this is the name of an
        actual file containing valid JSON, the latter will be parsed.
    """
    if isinstance(x, dict):
        return x
    if not isinstance(x, (str, bytes, os.PathLike)):
        return json.loads(x)  # try this i guess

    # x is now known to be a string.
    file_error = None
    parse_error = None
    file_parse_error = None

    try:
        f = open(x, 'r')
    except BaseException as e:
        file_error = e
    else:
        # x is a valid file and can be opened
        file_error = False
        try:
            return json.loads(f.read())
        except BaseException as e:
            file_parse_error = e
    finally:
        try:
            f.close()
        except BaseException:
            pass

    # if we're here, either file read or file JSON parsing failed.

    try:
        return json.loads(x)
    except BaseException as e:
        parse_error = e

    # if we're here, parsing x:str as JSON failed and x is not a valid JSON file.
    errors = list(filter(bool, [parse_error, file_error, file_parse_error]))
    try:
        ExceptionGroup
    except NameError:  # this python doesn't have ExceptionGroup
        raise Exception(errors)
    else:
        raise ExceptionGroup("Tried resolving as JSON string or as path to file but errors ocurred.", errors)  # new in python 3.11


class ValidJSONFilePath(os.PathLike):
    pass

class ValidJSONStr(str):
    pass


def json_detect(x) -> type:
    """
    Return the type of the input

    args:
    x should be a string containing valid JSON, or a file descriptor (path string).

    NB: Interpreting x as a file takes precedence over interpreting x
        as a JSON str. If x is '{"a": null}' and this is the name of an
        actual file containing valid JSON, the latter will be parsed.
    """
    if isinstance(x, dict) or not isinstance(x, (str, bytes, os.PathLike)):
        return type(x)

    # x is now known to be a str or pathlike.
    file_error = None
    parse_error = None
    file_parse_error = None

    try:
        f = open(x, 'r')
    except BaseException as e:
        file_error = e
    else:
        # x is a valid file and can be opened
        file_error = False
        try:
            json.loads(f.read())
            return ValidJSONFilePath
        except BaseException as e:
            file_parse_error = e
    finally:
        try:
            f.close()
        except BaseException:
            pass

    # if we're here, either file read or file JSON parsing failed.

    try:
        json.loads(x)
        return ValidJSONStr
    except BaseException as e:
        parse_error = e

    # if we're here, parsing x:str as JSON failed and x is not a valid JSON file.
    errors = list(filter(bool, [parse_error, file_error, file_parse_error]))
    try:
        ExceptionGroup
    except NameError:  # this python doesn't have ExceptionGroup
        raise Exception(errors)
    else:
        raise ExceptionGroup("Tried resolving as JSON string or as path to file but errors ocurred.", errors)  # new in python 3.11
