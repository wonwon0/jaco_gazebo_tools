file(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/joint_states_listener_real_jaco/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rospack_genmsg_libexe.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
