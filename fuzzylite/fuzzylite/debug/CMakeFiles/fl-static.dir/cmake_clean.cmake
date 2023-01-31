file(REMOVE_RECURSE
  "bin/libfuzzylite-static-debug.a"
  "bin/libfuzzylite-static-debug.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/fl-static.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
