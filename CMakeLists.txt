cmake_minimum_required(VERSION 3.16)
project(statistics VERSION 0.1.0)

# abilita il supporto per i test, tra cui l'opzione BUILD_TESTING usata sotto
include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# richiedi l'uso di C++17, senza estensioni non-standard offerte dal compilatore usato 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# abilita warning
string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra")

# Trova SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(boids boid.cpp main.cpp flock.cpp vector.cpp)

# Aggiungi SFML alle librerie da collegare
target_link_libraries(boids PRIVATE sfml-graphics sfml-window sfml-system)

# abilita:
# - l'address sanitizer e l'undefined-behaviour sanitizer
# - _LIBCPP_ENABLE_ASSERTIONS=1 (utilizzato da clang)
# - _GLIBCXX_ASSERTIONS (utilizzato da gcc)
#in debug mode
string(APPEND CMAKE_CXX_FLAGS_DEBUG " -D_GLIBCXX_ASSERTIONS -D_LIBCPP_ENABLE_ASSERTIONS=1 -fsanitize=address,undefined -fno-omit-frame-pointer")
string(APPEND CMAKE_EXE_LINKER_FLAGS_DEBUG " -fsanitize=address,undefined -fno-omit-frame-pointer")

# se il testing e' abilitato...
#   per disabilitare il testing, passare -DBUILD_TESTING=OFF a cmake durante la fase di configurazione
if (BUILD_TESTING)

    # Aggiungi gli eseguibili di test
    add_executable(flock.t flock.cpp boid.cpp vector.cpp flock.test.cpp)
    add_executable(boid.t boid.cpp vector.cpp boid.test.cpp)
    add_executable(vector.t vector.cpp vector.test.cpp)
    # Collega SFML agli eseguibili di test
    target_link_libraries(flock.t PRIVATE sfml-graphics sfml-window sfml-system)
    target_link_libraries(boid.t PRIVATE sfml-graphics sfml-window sfml-system)
    target_link_libraries(vector.t PRIVATE sfml-graphics sfml-window sfml-system)
 
    # Aggiungi gli eseguibili di test alla lista dei test
    add_test(NAME flock.t COMMAND flock.t)
    add_test(NAME boid.t COMMAND boid.t)
    add_test(NAME vector.t COMMAND vector.t)

endif()

