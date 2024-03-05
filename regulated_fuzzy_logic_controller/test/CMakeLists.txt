# tests for test_regulated_fuzzy_logic
ament_add_gtest(test_regulated_fuzzy_logic
  test_regulated_fuzzy_logic.cpp
)
ament_target_dependencies(test_regulated_fuzzy_logic
  ${dependencies}
)
target_link_libraries(test_regulated_fuzzy_logic 
  ${library_name}
)