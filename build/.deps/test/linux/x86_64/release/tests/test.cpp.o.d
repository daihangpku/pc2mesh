{
    files = {
        "tests/test.cpp"
    },
    depfiles_gcc = "test.o: tests/test.cpp\
",
    values = {
        "/usr/bin/gcc",
        {
            "-m64",
            "-fvisibility=hidden",
            "-fvisibility-inlines-hidden",
            "-O3",
            "-std=c++20",
            "-isystem",
            "/home/daihang/.xmake/packages/e/eigen/3.4.0/c95d81810dbc45229e675130ada7d4c7/include",
            "-isystem",
            "/home/daihang/.xmake/packages/e/eigen/3.4.0/c95d81810dbc45229e675130ada7d4c7/include/eigen3",
            "-DNDEBUG"
        }
    }
}