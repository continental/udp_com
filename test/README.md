# Testing Commands Overview
* `catkin_make run_tests` Runs the tests -- And Builds TOO!!
* `catkin_make tests` builds tests
* `catkin_make test` Runs tests also, but I am not sure what the difference is since there's different output

## gtest 
* TestCases are grouped into testSuites
* both are named via camelCase
## CMakeLists.txt
* Adding a test:
```
catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
```
* Links all of the tests (I believe) --only needed once
```
if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
```