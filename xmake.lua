set_project("pc2mesh")
set_version("0.1.0")

-- 设置编译选项
set_languages("c++20")
add_rules("mode.debug", "mode.release")

-- 添加依赖包
add_requires("glad")
add_requires("glfw")
add_requires("glm")
add_requires("imgui")
add_requires("spdlog")
add_requires("stb")
add_requires("fmt")
add_requires("tinyobjloader")
add_requires("yaml-cpp")
add_requires("eigen")

-- 主程序目标
target("bpa")
    set_kind("binary")
    add_files("src/*/*.cpp")
    
    -- 添加包依赖
    add_packages("eigen")
    add_packages("glfw")
    add_packages("glad")
    add_packages("imgui")
    
    -- 添加头文件路径
    add_includedirs("src")

-- 测试目标
target("test")
    set_kind("binary")
    add_files("tests/*.cpp")
    add_includedirs("src")
    add_deps("bpa")
    add_packages("eigen")