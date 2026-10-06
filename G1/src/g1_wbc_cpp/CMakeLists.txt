cmake_minimum_required(VERSION 3.16)
project(g1_wbc_cpp)

if(NOT CMAKE_BUILD_TYPE)
  # Khong dat Release thi Eigen chay cham hang chuc lan - ma muc dich cua ban
  # C++ nay chinh la toc do, nen de mac dinh Debug la tu phan bo chinh minh.
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O3)

# Build kiem tra bo nho:
#   colcon build --packages-select g1_wbc_cpp --cmake-args -DG1_WBC_ASAN=ON
# Cham hon vai lan nen KHONG dung de do thoi gian - chi de tim cho ghi tran.
option(G1_WBC_ASAN "bat AddressSanitizer" OFF)
if(G1_WBC_ASAN)
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=address)
  # BAT BUOC. Eigen co dong nay trong Memory.h:
  #     ... && ! defined(__SANITIZE_ADDRESS__) && (EIGEN_DEFAULT_ALIGN_BYTES == 16)
  #       -> EIGEN_GLIBC_MALLOC_ALREADY_ALIGNED 1
  # tuc duoi ASan, Eigen DOI sang handmade_aligned_malloc/free, trong khi
  # libpinocchio_parsers.so (khong ASan) van dung malloc thuong. Vung nho do
  # thu vien cap phat se bi code cua ta giai phong bang handmade_aligned_free -
  # ham nay doc 8 byte PHIA TRUOC con tro. ASan bao heap-buffer-overflow ngay
  # o moi lan huy Model. Do la TAO TAC CUA CHINH ASan, khong phai loi that,
  # va no che mat loi can tim. Ep lai cho khop thu vien.
  add_compile_definitions(EIGEN_MALLOC_ALREADY_ALIGNED=1)
  message(STATUS "g1_wbc_cpp: AddressSanitizer BAT (da ep EIGEN_MALLOC_ALREADY_ALIGNED=1)")
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(proxsuite REQUIRED)
find_package(yaml-cpp REQUIRED)

# eiquadprog: tap tich cuc (Goldfarb-Idnani), cung thuat toan voi quadprog cua
# ban Python. Khong co trong apt neu khong sudo nen dung nguon tai cho. LGPL-3:
# build thanh thu vien CHIA SE rieng va lien ket dong, khong chep nguon vao repo.
set(EIQUADPROG_DIR "$ENV{HOME}/eiquadprog" CACHE PATH "thu muc nguon eiquadprog")
if(EXISTS "${EIQUADPROG_DIR}/src/eiquadprog-fast.cpp")
  # Chi lay ban "fast" (API dang dung). eiquadprog.cpp la API cu, keo theo
  # nhieu header do cmake sinh ra ma ta khong dung den.
  add_library(eiquadprog SHARED ${EIQUADPROG_DIR}/src/eiquadprog-fast.cpp)
  target_include_directories(eiquadprog
    PUBLIC $<BUILD_INTERFACE:${EIQUADPROG_DIR}/include>
           $<INSTALL_INTERFACE:include>
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party_shim>)
  target_link_libraries(eiquadprog PUBLIC Eigen3::Eigen)
  target_compile_options(eiquadprog PRIVATE -w)   # nguon ngoai, khong soi canh bao
  install(TARGETS eiquadprog EXPORT export_${PROJECT_NAME} DESTINATION lib)
  message(STATUS "g1_wbc_cpp: co eiquadprog -> bat bo giai tap tich cuc")
else()
  message(WARNING "g1_wbc_cpp: khong thay eiquadprog tai ${EIQUADPROG_DIR} "
                  "-> chi co ProxQP")
endif()

add_library(balance_wbc src/balance_wbc.cpp src/leg_odometry.cpp src/safety.cpp)
if(TARGET eiquadprog)
  target_link_libraries(balance_wbc PUBLIC eiquadprog)
  target_compile_definitions(balance_wbc PUBLIC G1_WBC_HAS_EIQUADPROG)
endif()
target_include_directories(balance_wbc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(balance_wbc PUBLIC
  Eigen3::Eigen pinocchio::pinocchio proxsuite::proxsuite yaml-cpp)

add_executable(test_safety src/test_safety.cpp)
target_link_libraries(test_safety PRIVATE balance_wbc)
install(TARGETS test_safety DESTINATION lib/${PROJECT_NAME})

add_executable(run_test_vectors src/run_test_vectors.cpp)
target_link_libraries(run_test_vectors PRIVATE balance_wbc)

install(TARGETS balance_wbc EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# --- node che do bong: can SDK C++ cua Unitree ---
# SDK khong co trong apt nen de TUY CHON: khong tim thay thi van build phan con
# lai (thu vien + chuong trinh doi chieu), chi bo node song.
set(UNITREE_SDK2_DIR "$ENV{HOME}/unitree_sdk2" CACHE PATH "thu muc unitree_sdk2")
if(EXISTS "${UNITREE_SDK2_DIR}/lib/x86_64/libunitree_sdk2.a")
  # shadow_node chi DOC; handover_node la chuong trinh DUY NHAT co the lam
  # robot chuyen dong (va mac dinh no cung khong gui gi).
  foreach(tgt shadow_node handover_node)
    if(tgt STREQUAL "handover_node")
      add_executable(${tgt} src/handover_node.cpp src/lowcmd.cpp)
    else()
      add_executable(${tgt} src/${tgt}.cpp)
    endif()
    target_include_directories(${tgt} PRIVATE
      ${UNITREE_SDK2_DIR}/include
      ${UNITREE_SDK2_DIR}/thirdparty/include
      ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx
      ${UNITREE_SDK2_DIR}/thirdparty/include/ddsc)
    target_link_libraries(${tgt} PRIVATE balance_wbc
      ${UNITREE_SDK2_DIR}/lib/x86_64/libunitree_sdk2.a
      ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64/libddsc.so
      ${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64/libddscxx.so
      pthread)
    set_target_properties(${tgt} PROPERTIES
      BUILD_RPATH "${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64"
      INSTALL_RPATH "${UNITREE_SDK2_DIR}/thirdparty/lib/x86_64")
    install(TARGETS ${tgt} DESTINATION lib/${PROJECT_NAME})
  endforeach()
  message(STATUS "g1_wbc_cpp: co unitree_sdk2 -> build shadow_node")
else()
  message(WARNING "g1_wbc_cpp: khong thay unitree_sdk2 tai ${UNITREE_SDK2_DIR} "
                  "-> BO QUA shadow_node (phan con lai van build)")
endif()

install(TARGETS run_test_vectors DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio proxsuite yaml-cpp)
ament_package()
