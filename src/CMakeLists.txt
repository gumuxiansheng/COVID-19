if (NOT BUILD_CXX)
    return()
endif()

project(covid19_or)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/..)

foreach(EXECUTABLE
  cvrp_raw
  ccvrp_raw
  ccvrp_multi_depot_raw)
  add_executable(${EXECUTABLE} ${EXECUTABLE}.cc utils/file_util.cc utils/distance_util.cc utils/requirements_util.cc utils/vector_help.cc algorithms/vns.cc algorithms/vns.h algorithms/init_solution.cc algorithms/init_solution.h algorithms/cost.cc algorithms/cost.h)
  target_include_directories(${EXECUTABLE} PUBLIC ${PARENT_SOURCE_DIR})
  target_compile_features(${EXECUTABLE} PRIVATE cxx_std_11)
endforeach()

set(USE_OR_TOOLS FALSE)

if (NOT USE_OR_TOOLS)
  return()
endif()

set(ORTOOLS_BUILD_DIR ${PROJECT_SOURCE_DIR}/../ortoolsbuild/usr/local)

foreach(EXECUTABLE
  vrp
  cvrp)
  add_executable(${EXECUTABLE} ${EXECUTABLE}.cc utils/file_util.cc utils/distance_util.cc utils/requirements_util.cc utils/vector_help.cc algorithms/vns.cc algorithms/vns.h)
  target_include_directories(${EXECUTABLE} PUBLIC ${PARENT_SOURCE_DIR})
  target_compile_features(${EXECUTABLE} PRIVATE cxx_std_11)
  target_include_directories(${EXECUTABLE} PUBLIC ${ORTOOLS_BUILD_DIR}/include)
  target_link_directories(${EXECUTABLE} PUBLIC ${ORTOOLS_BUILD_DIR}/lib)
  target_link_libraries(${EXECUTABLE} PUBLIC -L${ORTOOLS_BUILD_DIR}/lib -lgflags -L${ORTOOLS_BUILD_DIR}/lib -lglog -L${ORTOOLS_BUILD_DIR}/lib/ -lprotobuf ${ORTOOLS_BUILD_DIR}/lib/libabsl_synchronization.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_bad_any_cast_impl.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_bad_optional_access.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_bad_variant_access.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_city.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_civil_time.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_examine_stack.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_failure_signal_handler.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_config.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_marshalling.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_parse.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_program_name.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_registry.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_usage.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_flags_usage_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_graphcycles_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_hash.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_hashtablez_sampler.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_leak_check.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_leak_check_disable.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_log_severity.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_malloc_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_periodic_sampler.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_distributions.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_internal_distribution_test_util.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_internal_pool_urbg.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_internal_randen.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_internal_randen_hwaes.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_internal_randen_hwaes_impl.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_internal_randen_slow.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_internal_seed_material.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_seed_gen_exception.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_random_seed_sequences.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_raw_hash_set.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_raw_logging_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_scoped_set_env.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_spinlock_wait.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_stacktrace.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_str_format_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_strings.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_strings_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_symbolize.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_throw_delegate.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_time.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_time_zone.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_exponential_biased.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_int128.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_debugging_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_demangle_internal.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_base.a ${ORTOOLS_BUILD_DIR}/lib/libabsl_dynamic_annotations.a -L${ORTOOLS_BUILD_DIR}/lib -lCbcSolver -lCbc -lOsiCbc -L${ORTOOLS_BUILD_DIR}/lib -lCgl -L${ORTOOLS_BUILD_DIR}/lib -lClpSolver -lClp -lOsiClp -L${ORTOOLS_BUILD_DIR}/lib -lOsi -L${ORTOOLS_BUILD_DIR}/lib -lCoinUtils -Llib -lortools)
endforeach()