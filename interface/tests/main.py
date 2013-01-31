import unittest
import tests.verifications.steering

def all_suites():
    suites = []
    suites.append(tests.verifications.steering.suite())
    return unittest.TestSuite(suites)

def run():
    suites = all_suites()
    unittest.TextTestRunner().run(suites)

