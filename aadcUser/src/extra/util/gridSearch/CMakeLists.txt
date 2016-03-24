cmake_minimum_required(VERSION 2.8.11)
project(GridSearch)

add_executable(GridSearch
    main.cpp
    GridSearch.h
    GridSearch.cpp
    SearchMap.h
    SearchMap.cpp
    SearchNode.h
    SearchNode.cpp
    CompareSearchNode.h
    TestMap.h
    TestMap.cpp
)