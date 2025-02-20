check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/FirstOrderDeterministicPathLoss-examples-test-suite.cc
        )
endif()

build_lib(
    LIBNAME FirstOrderBuildingsAwarePathLoss
    SOURCE_FILES model/first-order-buildings-aware-propagation-loss-model.cc
                 model/foba-toolbox.cc
    HEADER_FILES model/first-order-buildings-aware-propagation-loss-model.h
                 model/foba-toolbox.h
    LIBRARIES_TO_LINK ${libmobility}
    ${libbuildings}
    ${libpropagation}
    TEST_SOURCES test/first-order-deterministic-path-loss-test-suite.cc
                 ${examples_as_tests_sources}
)
