if(CMAKE_CXX_COMPILER MATCHES "/em\\+\\+(-[a-zA-Z0-9.])?(\\.bat)?$")
  set(CMAKE_CXX_COMPILER_ID "Emscripten")
endif()
