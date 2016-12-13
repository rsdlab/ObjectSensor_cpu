set(hdrs ObjectSensor_ORB.h
ImageViewer.h
 TimeStampUtil_forRTC.h
 SIFT.hpp
 SIFTMatching.hpp
 SIFTPatternFinder.hpp
 SIFTPatternFinderStereo.hpp
 siftpp_convolve.hpp
 siftpp_vore.hpp
 siftpp_defs.h
 siftpp_xform.h
 Camera.hpp
 CameraStereo.hpp
 CMinpack.hpp
 cuda_image.h
 cuda_image_edit.h
 cuda_image_filter.h
 cuda_sift.h
 geometry3d.hpp
 guih_config.hpp
 ImageViewer.h
 imgh_common.hpp
 imgh_converter.hpp
 imgh_drawer.hpp
 imgh_editor.hpp
 imgh_file_io.hpp
 imgh_filter.hpp
 lm.h
 misc.h
 mth_homography.hpp
 mth_rotation.hpp
 mtx_jama_cholesky.h
 mtx_jama_eig.h
 mtx_jama_lu.h
 mtx_jama_qr.h
 mtx_jama_svd.h
 mtx_matrix.hpp
 mtx_matrix_solver.hpp
 mtx_tnt_array1d.h
 mtx_tnt_array2d.h
 mtx_tnt_i_refvec.h
 mtx_tnt_math_utils.h
 MultiCastEventListener.h
 util_timer.hpp
 vm_macros.h
    PARENT_SCOPE
    AcceptModelSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

