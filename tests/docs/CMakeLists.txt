# Documentation checks (tools/check_docs.py): relative links and heading
# anchors in README.md, README.tr.md and docs/, the physics formula index
# (docs/physics_index.json against docs/physics.md and the sources), and the
# coverage of every APPROXIMATION marker under src/. No build output is read.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME docs.check
        COMMAND Python3::Interpreter ${PROJECT_SOURCE_DIR}/tools/check_docs.py)
    # The checker's own cases: links that must pass and links that must fail.
    add_test(NAME docs.check_selftest
        COMMAND Python3::Interpreter ${PROJECT_SOURCE_DIR}/tests/docs/test_check_docs.py)
    # tools/make_screenshots.py spawns the --app it was given, by absolute path.
    add_test(NAME docs.make_screenshots_selftest
        COMMAND Python3::Interpreter ${PROJECT_SOURCE_DIR}/tests/docs/test_make_screenshots.py)
else()
    message(WARNING "Python 3 not found: docs.check test not registered")
endif()
