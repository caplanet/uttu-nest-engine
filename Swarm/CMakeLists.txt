cmake_minimum_required(VERSION 3.26)
# This project only orchestrates Cargo/custom targets and installs generated
# artifacts/headers; requiring a C compiler makes otherwise valid Rust/WPF
# environments fail during configure for no functional reason.
project(swarm LANGUAGES NONE)

# -- Find Rust / Cargo
# Search PATH first, then the standard rustup install location, so a freshly
# installed toolchain is found even when PATH hasn't been refreshed yet
# (e.g. Visual Studio / the shell was opened before Rust was installed).
find_program(CARGO_EXECUTABLE cargo
  PATHS
    "$ENV{CARGO_HOME}/bin"
    "$ENV{USERPROFILE}/.cargo/bin"
    "$ENV{HOME}/.cargo/bin"
)
if(NOT CARGO_EXECUTABLE)
  message(FATAL_ERROR
    "cargo not found.\n"
    "  1) Install Rust:  https://rustup.rs   (or:  winget install Rustlang.Rustup)\n"
    "     then:  rustup toolchain install nightly\n"
    "  2) Make sure cargo is on PATH (rustup adds %USERPROFILE%\\.cargo\\bin).\n"
    "  3) RESTART the shell / fully restart Visual Studio so the new PATH is seen.\n"
    "  4) DELETE the CMake cache and reconfigure (VS: 'Delete Cache and Reconfigure',\n"
    "     or remove the build/out directory) — CMake caches this not-found result.")
endif()
message(STATUS "Found cargo: ${CARGO_EXECUTABLE}")

set(CARGO_MANIFEST_DIR "${CMAKE_SOURCE_DIR}")

# -- Helper: detect target triple
if(WIN32)
  set(SWARM_TARGET_PROCESSOR "${CMAKE_GENERATOR_PLATFORM}")
  if(NOT SWARM_TARGET_PROCESSOR)
    set(SWARM_TARGET_PROCESSOR "$ENV{PROCESSOR_ARCHITEW6432}")
  endif()
  if(NOT SWARM_TARGET_PROCESSOR)
    set(SWARM_TARGET_PROCESSOR "$ENV{PROCESSOR_ARCHITECTURE}")
  endif()
  string(TOLOWER "${SWARM_TARGET_PROCESSOR}" SWARM_TARGET_PROCESSOR)
  if(SWARM_TARGET_PROCESSOR MATCHES "arm64|aarch64")
    set(RUST_TARGET_TRIPLE "aarch64-pc-windows-msvc")
  elseif(SWARM_TARGET_PROCESSOR MATCHES "x64|amd64|x86_64")
    set(RUST_TARGET_TRIPLE "x86_64-pc-windows-msvc")
  else()
    set(RUST_TARGET_TRIPLE "i686-pc-windows-msvc")
  endif()
elseif(APPLE)
  set(SWARM_TARGET_PROCESSOR "${CMAKE_SYSTEM_PROCESSOR}")
  if(NOT SWARM_TARGET_PROCESSOR)
    set(SWARM_TARGET_PROCESSOR "${CMAKE_HOST_SYSTEM_PROCESSOR}")
  endif()
  if(SWARM_TARGET_PROCESSOR MATCHES "arm64|aarch64")
    set(RUST_TARGET_TRIPLE "aarch64-apple-darwin")
  else()
    set(RUST_TARGET_TRIPLE "x86_64-apple-darwin")
  endif()
else()
  set(SWARM_TARGET_PROCESSOR "${CMAKE_SYSTEM_PROCESSOR}")
  if(NOT SWARM_TARGET_PROCESSOR)
    set(SWARM_TARGET_PROCESSOR "${CMAKE_HOST_SYSTEM_PROCESSOR}")
  endif()
  string(TOLOWER "${SWARM_TARGET_PROCESSOR}" SWARM_TARGET_PROCESSOR)
  if(SWARM_TARGET_PROCESSOR MATCHES "arm64|aarch64")
    set(RUST_TARGET_TRIPLE "aarch64-unknown-linux-gnu")
  elseif(SWARM_TARGET_PROCESSOR MATCHES "x64|amd64|x86_64")
    set(RUST_TARGET_TRIPLE "x86_64-unknown-linux-gnu")
  else()
    set(RUST_TARGET_TRIPLE "i686-unknown-linux-gnu")
  endif()
endif()

message(STATUS "Rust target triple: ${RUST_TARGET_TRIPLE}")

# -- Build type mapping (works for multi-config generators too)
# Visual Studio is a MULTI-CONFIG generator: CMAKE_BUILD_TYPE is empty at
# configure time and the real config (Debug/Release) is only known at BUILD
# time. Use a generator expression so cargo's profile, its output directory,
# and the copy step below all agree — on single-config (Make/Ninja) AND
# multi-config (Visual Studio) generators. The custom `dev-fast` profile emits
# into target/dev-fast/, so the install paths must use that same directory.
set(CARGO_BUILD_DIR "$<IF:$<CONFIG:Debug>,dev-fast,release>")
set(CARGO_PROFILE_FLAG "--profile" "$<IF:$<CONFIG:Debug>,dev-fast,release>")

# -- Output paths
set(CARGO_OUT_DIR "${CARGO_MANIFEST_DIR}/target/${CARGO_BUILD_DIR}")

# Cargo output file names are platform-specific:
if(WIN32)
  set(CARGO_DLL_NAME       "cpswcore.dll")
  set(CARGO_STATICLIB_NAME "cpswcore.lib")
  set(CARGO_IMPORTLIB_NAME "cpswcore.dll.lib")

  set(DLL_FILE        "${CARGO_OUT_DIR}/${CARGO_DLL_NAME}")
  set(LIB_FILE        "${CARGO_OUT_DIR}/${CARGO_IMPORTLIB_NAME}")
  set(STATIC_LIB_FILE "${CARGO_OUT_DIR}/${CARGO_STATICLIB_NAME}")
elseif(APPLE)
  set(CARGO_DLL_NAME       "libcpswcore.dylib")
  set(CARGO_STATICLIB_NAME "libcpswcore.a")

  set(DLL_FILE        "${CARGO_OUT_DIR}/${CARGO_DLL_NAME}")
  set(STATIC_LIB_FILE "${CARGO_OUT_DIR}/${CARGO_STATICLIB_NAME}")
else()
  set(CARGO_DLL_NAME       "libcpswcore.so")
  set(CARGO_STATICLIB_NAME "libcpswcore.a")

  set(DLL_FILE        "${CARGO_OUT_DIR}/${CARGO_DLL_NAME}")
  set(STATIC_LIB_FILE "${CARGO_OUT_DIR}/${CARGO_STATICLIB_NAME}")
endif()

# -- Collect Rust source files for IDE visibility
file(GLOB_RECURSE RUST_SOURCES "${CARGO_MANIFEST_DIR}/src/*.rs")
file(GLOB_RECURSE WASM_SOURCES "${CARGO_MANIFEST_DIR}/swarm-wasm/src/*.rs")
list(APPEND RUST_PROJECT_FILES
  "${CARGO_MANIFEST_DIR}/Cargo.toml"
  "${CARGO_MANIFEST_DIR}/swarm-wasm/Cargo.toml"
  ${RUST_SOURCES}
  ${WASM_SOURCES}
)

# Make Rust sources visible in Visual Studio's Solution Explorer
if(MSVC OR (CMAKE_GENERATOR STREQUAL "Visual Studio 18 2026" OR CMAKE_GENERATOR MATCHES "Visual Studio"))
  # Create a source group tree starting from the src dir, so VS shows nested folders
  source_group(TREE "${CARGO_MANIFEST_DIR}/src" PREFIX "Rust\\src" FILES ${RUST_SOURCES})
  source_group("Rust" FILES "${CARGO_MANIFEST_DIR}/Cargo.toml" "${CARGO_MANIFEST_DIR}/Cargo.lock")
endif()

# -- Options
option(BUILD_WITH_JNI "Build swarm with JNI support (requires JDK)" ON)

# Keep every native entry point used by the managed/WPF client in the standard
# CMake build.  In particular, swarm_gp_phase is present as a symbol
# even without the structure-first feature, but returns an unsupported-feature
# error at runtime; using one shared feature list prevents that silent mismatch.
set(SWARM_CARGO_FEATURE_LIST simd capmods adaptive structure-first)
if(BUILD_WITH_JNI)
  list(APPEND SWARM_CARGO_FEATURE_LIST jni)
endif()
list(JOIN SWARM_CARGO_FEATURE_LIST "," SWARM_CARGO_FEATURES)
message(STATUS "Swarm cargo features: ${SWARM_CARGO_FEATURES}")

# -- Cargo build (cdylib + staticlib)
# ALL target — Build Solution triggers cargo build.
# ${CARGO_PROFILE_FLAG} expands to --profile dev-fast (Debug) or --profile release (Release).
add_custom_target(cargo-build ALL
  COMMAND "${CARGO_EXECUTABLE}" build ${CARGO_PROFILE_FLAG} --features "${SWARM_CARGO_FEATURES}"
  WORKING_DIRECTORY "${CARGO_MANIFEST_DIR}"
  COMMENT "Building swarm Rust library"
)

# -- Convenience targets
add_custom_target(cargo-build-debug
  COMMAND "${CARGO_EXECUTABLE}" build --profile dev-fast --features "${SWARM_CARGO_FEATURES}"
  WORKING_DIRECTORY "${CARGO_MANIFEST_DIR}"
  COMMENT "Build Rust debug binaries with cargo"
)

add_custom_target(cargo-build-release
  COMMAND "${CARGO_EXECUTABLE}" build --release --features "${SWARM_CARGO_FEATURES}"
  WORKING_DIRECTORY "${CARGO_MANIFEST_DIR}"
  COMMENT "Build Rust release binaries with cargo"
)

# wasm-pack profile flag — matches Visual Studio config
set(WASM_PROFILE_FLAG "$<IF:$<CONFIG:Debug>,--dev,--release>")
# javac debug flag — debug info on/off matching VS config
set(JAVA_DEBUG_FLAG "$<IF:$<CONFIG:Debug>,-g,-g:none>")

# -- WASM build (wasm-pack)
find_program(WASM_PACK_EXECUTABLE wasm-pack
  PATHS
    "$ENV{CARGO_HOME}/bin"
    "$ENV{USERPROFILE}/.cargo/bin"
    "$ENV{HOME}/.cargo/bin"
)
if(NOT WASM_PACK_EXECUTABLE)
  message(WARNING
    "wasm-pack not found. Install with:  cargo install wasm-pack\n"
    "The 'wasm-build' target will not be available."
  )
else()
  message(STATUS "Found wasm-pack: ${WASM_PACK_EXECUTABLE}")

# -- add_custom_target(wasm-build ALL
  add_custom_target(wasm-build 
    COMMAND "${WASM_PACK_EXECUTABLE}" build swarm-wasm --target web
            ${WASM_PROFILE_FLAG}
            --out-dir "${CARGO_MANIFEST_DIR}/target/${CARGO_BUILD_DIR}/wasm"
    WORKING_DIRECTORY "${CARGO_MANIFEST_DIR}"
    COMMENT "Build swarm-wasm for browser/web → target/${CARGO_BUILD_DIR}/wasm/"
  )

# -- add_custom_target(wasm-build-bundler ALL
  add_custom_target(wasm-build-bundler 
    COMMAND "${WASM_PACK_EXECUTABLE}" build swarm-wasm --target bundler
            ${WASM_PROFILE_FLAG}
            --out-dir "${CARGO_MANIFEST_DIR}/target/${CARGO_BUILD_DIR}/wasm"
    WORKING_DIRECTORY "${CARGO_MANIFEST_DIR}"
    COMMENT "Build swarm-wasm for Node.js / bundler → target/${CARGO_BUILD_DIR}/wasm/"
  )
endif()

# -- Java JNI bindings
find_program(JAVA_COMPILE javac
  PATHS
    "$ENV{JAVA_HOME}/bin"
    "C:/Program Files/Android/openjdk"
    "C:/Program Files/Java"
    "C:/Program Files/Eclipse Adoptium"
    "C:/Program Files/Microsoft"
    PATH_SUFFIXES bin
)
if(NOT JAVA_COMPILE)
  message(WARNING
    "javac not found. Install JDK or set JAVA_HOME.\n"
    "The 'java-build' target will not be available."
  )
else()
  message(STATUS "Found javac: ${JAVA_COMPILE}")

# -- compile_java.bat : Detect javac path batch file
# -- add_custom_target(java-build ALL
  add_custom_target(java-build 
    COMMAND "${CARGO_MANIFEST_DIR}/compile_java.bat"
            "${CARGO_MANIFEST_DIR}/target/${CARGO_BUILD_DIR}/java"
            "${JAVA_COMPILE}"
            "${CARGO_MANIFEST_DIR}"
    WORKING_DIRECTORY "${CARGO_MANIFEST_DIR}"
    COMMENT "Compiling Java JNI bindings → target/${CARGO_BUILD_DIR}/java/"
  )
endif()

# -- Install rules
# GNUInstallDirs cannot infer lib vs lib64 without an enabled compiler
# language. Rust artifacts use the portable `lib` destination here.
set(CMAKE_INSTALL_LIBDIR "lib" CACHE PATH "Object code libraries")
include(GNUInstallDirs)

install(FILES "${DLL_FILE}"
  DESTINATION "${CMAKE_INSTALL_BINDIR}"
  COMPONENT runtime
)

install(FILES "${STATIC_LIB_FILE}"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  COMPONENT development
)

if(WIN32)
  install(FILES "${LIB_FILE}"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    COMPONENT development
  )
endif()

# ── Header copy ────────────────────────────────────────────────────
install(FILES "${CMAKE_SOURCE_DIR}/include/swarm.h"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT development
)
