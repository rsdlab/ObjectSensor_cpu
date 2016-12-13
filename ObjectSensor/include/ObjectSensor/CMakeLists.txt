set(hdrs ObjectSensor.h
    PARENT_SCOPE
    AcceptModelSVC_impl.h
    )

install(FILES ${hdrs} DESTINATION ${INC_INSTALL_DIR}/${PROJECT_NAME_LOWER}
    COMPONENT library)

