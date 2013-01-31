__author__="Eric Perko (exp63)"

import inspect

def add_test_cases(cls):
    values = {}
    functions = {}
    # Find all the 'multitest*' functions and
    # matching list of test values.
    for key, value in inspect.getmembers(cls):
        if key.startswith("multitest"):
            if key.endswith("_values"):
                values[key[:-7]] = value
            else:
                functions[key] = value

    # Put them together to make a list of new test functions.
    # One test function for each value
    for key in functions:
        if key in values:
            function = functions[key]
            for i, value in enumerate(values[key]):
                def test_function(self, function=function, value=value):
                    function(self, value)
                name ="test%s_%d" % (key[9:], i+1)
                test_function.__name__ = name
                setattr(cls, name, test_function)
