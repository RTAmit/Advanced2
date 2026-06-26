**Review Procedure**

In this exercise, submissions will be reviewed based on test coverage and performance. For each submission, a set of bugs will be inserted to your program. The test suite you wrote will be executed for each individual bug introduced.

We expect some test in the relevant component will catch the introduced bug while tests of other components will be unaffected**\***. Moreover, we will provide you with configuration and map files for integration tests, and those should catch some of the introduced bugs as well.

Catching \>50% of bugs on each component will reward with full marks, and catching \>25% of bugs will reward with partial marks.

Finally, the provided scenarios for integration tests should always finish in reasonable time (about 1 minute at most). Integration tests will also run without any introduced bugs and should finish successfully within 1 minute.

When introducing the bugs, some sections of your code might be manually reviewed. This manual review will be far less comprehensive than exercise 1, and will more likely result in comments without affecting the grade. However, cases of extremely poor coding practices will affect your grade.

**(\*)** Some cases can fail multiple components in a valid way, e.g. an especially destructive bug might fail tests in both SimulationRun and in SimulationManager categories. However, component tests are meant to isolate problems in specific locations in the program, so try to encapsulate tests when possible.

**Example**

The skeleton of exercise 2 already contains a reference implementation of MockLidar, yet you are expected to write tests for this component.

For example, a bug can be introduced where rays travel only 2/3rds of the z\_max value provided. Such a bug can be caught by a test verifying that obstacles at the very end of the beam are detected correctly, or other tests you might decide to perform.