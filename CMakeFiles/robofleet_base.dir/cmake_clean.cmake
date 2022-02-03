file(REMOVE_RECURSE
  "librobofleet_base.a"
  "librobofleet_base.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/robofleet_base.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
